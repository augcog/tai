import json
from threading import Thread, Lock
from typing import Any, Generator, List, Optional, Tuple

import transformers
from app.api.v1.services.rag_retriever import (
    clean_path,
    _get_reference_documents,
    _get_pickle_and_class,
    embedding_model
)
from app.core.models.chat_completion import Message
from pydantic import BaseModel

pipeline_generation_lock = Lock()


# class Message(BaseModel):
#     role: str
#     content: str


def is_local_pipeline(pipeline: Any) -> bool:
    return hasattr(pipeline, "tokenizer") and pipeline.tokenizer is not None


def generate_text_in_thread(messages: List[Message], streamer_iterator: Any, pipeline: Any = None) -> None:
    """
    Generate text via a local Hugging Face pipeline in a background thread.
    Uses tokenizer-based prompt generation if available; otherwise calls the pipeline with raw text.

    This behavior is to enable running model on local / mock mode / remote mode.
    """
    with pipeline_generation_lock:
        if is_local_pipeline(pipeline):
            # terminators = [
            #     pipeline.tokenizer.eos_token_id,
            #     pipeline.tokenizer.convert_tokens_to_ids("<|eot_id|>")
            # ]
            msg=[{"role": message.role, "content": message.content} for message in messages]
            prompt = pipeline.tokenizer.apply_chat_template(
                msg,
                tokenize=False,
                add_generation_prompt=True
            )
            pipeline(
                prompt,
                max_new_tokens=1024,
                do_sample=True,
                streamer=streamer_iterator
            )
        else:
            prompt = messages[-1].content
            pipeline(prompt, max_new_tokens=1000, do_sample=True)


def build_augmented_prompt(user_message: str, course: str, embedding_dir: str, threshold: float, rag: bool,top_k: int = 7
                           ) -> Tuple[str, List[str], str]:
    """
    Build an augmented prompt by retrieving reference documents.
    Returns:
      - modified_message: the augmented instruction prompt.
      - reference_list: list of reference URLs for JSON output.
      - reference_string: formatted string for plain text references.
    TODO: reference_string can be removed in the future once the legacy code migration is completed.
    """
    print("\nUser Question: \n", user_message, "\n")

    if not rag:
        return user_message, [], ""

    picklefile, class_name = _get_pickle_and_class(course)
    current_dir = embedding_dir
    query_embed = embedding_model.encode(
        user_message, return_dense=True, return_sparse=True, return_colbert_vecs=True
    )
    top_ids, top_docs, top_urls, similarity_scores,top_files,top_topic_paths = _get_reference_documents(query_embed, current_dir, picklefile,top_k=7)

    insert_document = ""
    reference_list: List[str] = []
    reference_string = ""
    n = 0
    print(top_ids)
    print(similarity_scores)
    for i in range(len(top_docs)):
        reference_list.append(top_urls[i] if top_urls[i] else "")
        if similarity_scores[i] > threshold:
            n += 1
            cleaned_info_path = top_ids[i]
            cleaned_file_path = top_files[i]
            cleaned_topic_path = top_topic_paths[i]
            if top_urls[i]:
                insert_document += (
                    f"\"\"\"Reference Number: {n}\n"
                    f"Directory Path to file: {cleaned_file_path}\n"
                    f"Topic Path of chunk in file: {cleaned_topic_path}\n"
                    f"Document: {top_docs[i]}\"\"\"\n\n"
                )
                reference_string += (
                    f"Reference {n}: <|begin_of_reference_name|>{cleaned_info_path}"
                    f"<|end_of_reference_name|><|begin_of_reference_link|>{top_urls[i]}"
                    f"<|end_of_reference_link|>\n\n"
                )
            else:
                insert_document += (
                    f"\"\"\"Reference Number: {n}\n"
                    f"Directory Path to file: {cleaned_file_path}\n"
                    f"Topic Path of chunk in file: {cleaned_topic_path}\n"
                    f"Document: {top_docs[i]}\"\"\"\n\n"
                )
                reference_string += (
                    f"Reference {n}: <|begin_of_reference_name|>{cleaned_info_path}"
                    f"<|end_of_reference_name|><|begin_of_reference_link|>"
                    f"<|end_of_reference_link|>\n\n"
                )

    if not insert_document or n == 0:
        modified_message = (
            f"Answer the instruction thoroughly with a well structured markdown format answer. If unsure of the answer, explain that there is no data in the knowledge base "
            f"for the response and refuse to answer. If the instruction is not related to any topic related to {class_name}, "
            f"explain and refuse to answer.\n---\n"
            f"Instruction: {user_message}"
        )
    else:
        insert_document += f"Instruction: {user_message}"
        modified_message = (
            f"Understand the reference documents and pick the helpful ones to answer the instruction thoroughly with a well structured markdown format answer. "
            f"Keep your answer grounded in the facts of the references that are relevant."
            f"Remember to refer to specific reference number inline with md *bold style*.Remember to refer to specific reference number inline with md *bold style*. Remember to refer to specific reference number inline with md *bold style*.Do not list reference at the end. Do not explain if the reference is not related to the question."
            f"If the instruction is not related to any topic related to {class_name}, explain and refuse to answer.\n"
            f"---\n{insert_document}"
        )
    print("\nAugmented Prompt: \n", modified_message, "\n")
    return modified_message, reference_list, reference_string


def local_parser(stream: Any, reference_string: str) -> Generator[str, None, None]:
    """
    Yield tokens from a text stream and append the reference block at the end.
    TODO: This function can be removed in the future once the legacy code migration is completed.
    """
    for chunk in stream:
        result = chunk.replace("---<|user|>", "")
        yield result if result is not None else ""
        print(result, end="")
    ref_block = f'\n\n<|begin_of_reference|>\n\n{reference_string}<|end_of_reference|>'
    yield ref_block
    print(ref_block)


def parse_token_stream_for_json(stream: Any) -> Generator[str, None, None]:
    """
    Yield tokens from a text stream (simplified version for JSON output).
    """
    for chunk in stream:
        result = chunk.replace("<|eot_id|>", "")
        yield result if result is not None else ""


def format_chat_msg(messages: List[Message]) -> List[Message]:
    """
    Format a conversation by prepending an initial system message.
    """
    response: List[Message] = []
    system_message = (
        "You are a Teaching Assistant. You are responsible for answering questions and providing guidance to students. "
        "Do not provide direct answers to homework questions. If the question is related to a class topic, "
        "provide guidance and resources to help the student answer the question. "
        "If the question is not related to a class topic, explain that you cannot provide an answer."
    )
    response.append(Message(role="system", content=system_message))
    for message in messages:
        response.append(Message(role=message.role, content=message.content))
    return response



def generate_chat_response(
        messages: List[Message],
        stream: bool = True,
        rag: bool = True,
        course: Optional[str] = None,
        embedding_dir: str = "/home/bot/localgpt/tai/ai_course_bot/ai_chatbot_backend/app/embedding/",
        threshold: float = 0.32,
        top_k: int = 7,
        pipeline: Any = None
) -> Tuple[Any, str]:
    """
    Build an augmented message with references and run LLM inference.
    Returns a tuple: (stream, reference_string)
    """
    user_message = messages[-1].content
    modified_message, _, reference_string = build_augmented_prompt(
        user_message, course if course else "", embedding_dir, threshold, rag, top_k
    )

    messages[-1].content = modified_message
    if is_local_pipeline(pipeline):
        streamer_iterator = transformers.TextIteratorStreamer(pipeline.tokenizer, skip_prompt=True)
        t = Thread(target=generate_text_in_thread, args=(messages, streamer_iterator, pipeline))
        t.start()
        return streamer_iterator, reference_string
    else:
        response = pipeline(messages[-1].content, stream=stream, course=course)
        return response, reference_string


def rag_json_stream_generator(
        messages: List[Message],
        stream: bool = True,
        rag: bool = True,
        course: Optional[str] = None,
        # TODO: Revise the default embedding_dir path. And put it into the environment variable for best practice.
        embedding_dir: str = "/home/bot/localgpt/tai/ai_course_bot/ai_chatbot_backend/app/embedding/",
        threshold: float = 0.38,
        top_k: int = 7,
        pipeline: Any = None
) -> Generator[str, None, None]:
    """
    Build an augmented message with references and produce a JSON streaming response.
    """
    user_message = messages[-1].content
    modified_message, reference_list, reference_string = build_augmented_prompt(
        user_message, course if course else "", embedding_dir, threshold, rag, top_k
    )
    messages[-1].content = modified_message
    if is_local_pipeline(pipeline):
        streamer_iterator = transformers.TextIteratorStreamer(pipeline.tokenizer, skip_prompt=True)
        t = Thread(target=generate_text_in_thread, args=(messages, streamer_iterator, pipeline))
        t.start()

        def stream_json_response() -> Generator[str, None, None]:
            for chunk in parse_token_stream_for_json(streamer_iterator):
                yield json.dumps({"type": "token", "data": chunk}) + "\n"
            yield json.dumps({"type": "final", "references": reference_list}) + "\n"

        return stream_json_response()
    else:
        remote_stream = pipeline(messages[-1].content, stream=stream, course=course)

        def stream_json_response() -> Generator[str, None, None]:
            for item in remote_stream:
                yield item
            yield json.dumps({"type": "final", "references": reference_list}) + "\n"

        return stream_json_response()
