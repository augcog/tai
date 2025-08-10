import json
from typing import Any, Generator, List, Optional, Tuple

from app.services.rag_retriever import (
    _get_reference_documents,
    _get_pickle_and_class,
    embedding_model,
)
from app.core.models.chat_completion import Message
from transformers import AutoTokenizer
from vllm import SamplingParams
import time

SAMPLING = SamplingParams(temperature=0.3, top_p=0.95, max_tokens=4096)
MODEL_ID = "THUDM/GLM-4-9B-0414"
tokenizer = AutoTokenizer.from_pretrained(MODEL_ID)


# class Message(BaseModel):
#     role: str
#     content: str


def is_local_engine(engine: Any) -> bool:
    return hasattr(engine, "is_running") and engine.is_running


def generate_streaming_response(messages: List[Message], engine: Any = None) -> Any:
    chat = [
        {"role": m.role, "content": m.content, "tool_call_id": m.tool_call_id}
        for m in messages
    ]
    prompt = tokenizer.apply_chat_template(
        chat, tokenize=False, add_generation_prompt=True
    )
    return engine.generate(prompt, SAMPLING, request_id=str(time.time_ns()))


def build_augmented_prompt(
    user_message: str,
    course: str,
    embedding_dir: str,
    threshold: float,
    rag: bool,
    top_k: int = 7,
    practice: bool = False,
    problem_content: Optional[str] = None,
    answer_content: Optional[str] = None,
    file_name: Optional[str] = None
) -> Tuple[str, List[str], str]:
    """
    Build an augmented prompt by retrieving reference documents.
    Returns:
      - modified_message: the augmented instruction prompt.
      - reference_list: list of reference URLs for JSON output.
      - reference_string: formatted string for plain text references.
    TODO: reference_string can be removed in the future once the legacy code migration is completed.
    """
    if practice:
        user_message = (
            f"Course problem:\n{problem_content}\n"
            f"Answer attempted by user:\n{answer_content}\n"
            f"Instruction: {user_message}"
        )
    print('\n Course: \n', course, '\n')
    print("\nUser Question: \n", user_message, "\n")
    print('time of the day:', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()), '\n')

    if not rag:
        return user_message, [], ""

    picklefile, class_name = _get_pickle_and_class(course)
    current_dir = embedding_dir
    start_time = time.time()
    query_embed = embedding_model.encode(
        user_message, return_dense=True, return_sparse=True, return_colbert_vecs=True
    )
    end_time = time.time()
    print(f"Embedding time: {end_time - start_time:.2f} seconds")
    (
        top_ids,
        top_docs,
        top_urls,
        similarity_scores,
        top_files,
        top_topic_paths,
    ) = _get_reference_documents(query_embed, current_dir, picklefile, top_k=top_k)

    insert_document = ""
    reference_list: List[str] = []
    reference_string = ""
    n = 0
    for i in range(len(top_docs)):
        # reference_list.append(top_urls[i] if top_urls[i] else "")
        if similarity_scores[i] > threshold:
            n += 1
            info_path = top_ids[i]
            if file_name:
                chunk_file=info_path.split('>')[0].strip()
                if chunk_file in file_name:
                    info_path = '>'.join(info_path.split('>')[1:])
            file_path = top_files[i]
            topic_path = top_topic_paths[i]
            url= top_urls[i] if top_urls[i] else ""
            insert_document += (
                f'Reference Number: {n}\n'
                f"Directory Path to file: {file_path}\n"
                f"Topic Path of chunk in file: {topic_path}\n"
                f'Document: {top_docs[i]}\n\n'
            )
            reference_string += (
                f"Reference {n}: <|begin_of_reference_name|>{info_path}<|end_of_reference_name|>"
                f"<|begin_of_reference_link|>{url}<|end_of_reference_link|>"
                f"<|begin_of_file_path|>{file_path}<|end_of_file_path|>"
                f"<|begin_of_index|>1<|end_of_index|>\n\n"
            )
            reference_list.append([info_path, url,file_path])

    if not insert_document or n == 0:
        modified_message = (
            f"Answer the instruction thoroughly with a well structured markdown format answer. Provide guidance instead of direct answers to Course problem when answer is being asked. If unsure of the answer, explain that there is no data in the knowledge base "
            f"for the response and refuse to answer. If the instruction is not within the scope of any topic related to {course}: {class_name}, or not some general query, "
            f"explain and refuse to answer.\n---\n"

        )
    else:
        modified_message = (
            f"Understand the reference documents and pick the helpful ones to answer the instruction thoroughly with a well structured markdown format answer but no need to add '```markdown'. "
            f"Keep your answer grounded in the facts of the references that are relevant. Provide guidance instead of direct answers to Course problem when answer is being asked. If unsure of the answer, explain that there is no data in the knowledge base for the response and refuse to answer. "
            f"Remember to refer to specific reference number inline with md *bold style*. Remember to refer to specific reference number inline with md *bold style*. Remember to refer to specific reference number inline with md *bold style*.Do not list reference at the end. Do not explain if the reference is not related to the question."
            f"If the instruction is not within the scope of any topic related to {course}: {class_name}, or not some general query, or mentioned in / related to any of the reference documents, explain and refuse to answer.\n---\n"
        )
    if not practice:
        modified_message += f"Instruction: {user_message}"
    else:
        modified_message += user_message
    return modified_message, reference_list, reference_string


async def local_parser(
    stream: Any, reference_string: str
) -> Generator[str, None, None]:
    """
    Yield tokens from a text stream and append the reference block at the end.
    TODO: This function can be removed in the future once the legacy code migration is completed.
    """
    print('Response:')
    previous_text = ""
    async for output in stream:
        text = output.outputs[0].text
        chunk = text[len(previous_text) :]
        yield chunk
        previous_text = text
        print(chunk, end="")
    ref_block = f"\n\n<|begin_of_reference|>\n\n{reference_string}<|end_of_reference|>"
    yield ref_block
    print(ref_block)


async def parse_token_stream_for_json(stream: Any) -> Generator[str, None, None]:
    """
    Yield tokens from a text stream (simplified version for JSON output).
    """
    previous_text = ""
    async for output in stream:
        text = output.outputs[0].text
        chunk = text[len(previous_text) :]
        yield chunk
        previous_text = text


def format_chat_msg(messages: List[Message]) -> List[Message]:
    """
    Format a conversation by prepending an initial system message.
    """
    response: List[Message] = []
    system_message = (
        "You are a Teaching Assistant called TAI. You are responsible for answering questions and providing guidance to students. "
        "Do not provide direct answers to homework questions. If the question is related to a class topic, "
        "provide guidance and resources to help the student answer the question. "
        "If the question is not related to a class topic, reference or some general greetings, explain that you cannot provide an answer."
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
    embedding_dir: str = "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/",
    threshold: float = 0.32,
    top_k: int = 7,
    engine: Any = None,
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
    if is_local_engine(engine):
        iterator = generate_streaming_response(messages, engine)
        return iterator, reference_string
    else:
        response = engine(messages[-1].content, stream=stream, course=course)
        return response, reference_string

def generate_practice_response(
    messages: List[Message],
    problem_content: str,
    answer_content: str,
    file_name: str,
    stream: bool = True,
    rag: bool = True,
    course: Optional[str] = None,
    embedding_dir: str = "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/",
    threshold: float = 0.32,
    top_k: int = 7,
    engine: Any = None,
) -> Tuple[Any, str]:
    """
    Build an augmented message with references and run LLM inference.
    Returns a tuple: (stream, reference_string)
    """
    user_message = messages[-1].content
    modified_message, _, reference_string = build_augmented_prompt(
        user_message, course if course else "", embedding_dir, threshold, rag, top_k,practice=True,problem_content=problem_content, answer_content=answer_content, file_name=file_name
    )

    messages[-1].content = modified_message
    if is_local_engine(engine):
        iterator = generate_streaming_response(messages, engine)
        return iterator, reference_string
    else:
        response = engine(messages[-1].content, stream=stream, course=course)
        return response, reference_string

def rag_json_stream_generator(
    messages: List[Message],
    stream: bool = True,
    rag: bool = True,
    course: Optional[str] = None,
    # TODO: Revise the default embedding_dir path. And put it into the environment variable for best practice.
    embedding_dir: str = "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/",
    threshold: float = 0.38,
    top_k: int = 7,
    engine: Any = None,
) -> Generator[str, None, None]:
    """
    Build an augmented message with references and produce a JSON streaming response.
    """
    user_message = messages[-1].content
    modified_message, reference_list, reference_string = build_augmented_prompt(
        user_message, course if course else "", embedding_dir, threshold, rag, top_k
    )
    messages[-1].content = modified_message
    if is_local_engine(engine):
        iterator = generate_streaming_response(messages, engine)

        def stream_json_response() -> Generator[str, None, None]:
            for chunk in parse_token_stream_for_json(iterator):
                yield json.dumps({"type": "token", "data": chunk}) + "\n"
            yield json.dumps({"type": "final", "references": reference_list}) + "\n"

        return stream_json_response()
    else:
        remote_stream = engine(messages[-1].content, stream=stream, course=course)

        def stream_json_response() -> Generator[str, None, None]:
            for item in remote_stream:
                yield item
            yield json.dumps({"type": "final", "references": reference_list}) + "\n"

        return stream_json_response()
