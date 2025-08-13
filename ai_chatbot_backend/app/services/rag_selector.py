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
import re

SAMPLING = SamplingParams(temperature=0.3, top_p=0.95, max_tokens=4096)
MODEL_ID = "THUDM/GLM-4-9B-0414"
tokenizer = AutoTokenizer.from_pretrained(MODEL_ID)


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
        threshold: float,
        rag: bool,
        top_k: int = 7,
        query_message: str = "",
        practice: bool = False,
        problem_content: Optional[str] = None,
        answer_content: Optional[str] = None,
        file_name: Optional[str] = None,
        audio_response: bool = False
) -> Tuple[str, List[str | Any]]:
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
        return user_message, []

    if not query_message:
        query_message = user_message
    (
        top_ids,
        top_docs,
        top_urls,
        similarity_scores,
        top_files,
        top_topic_paths,
    ),class_name = _get_reference_documents(query_message, course, top_k=top_k)

    insert_document = ""
    reference_list: List[str | Any] = []
    n = 0
    for i in range(len(top_docs)):
        # reference_list.append(top_urls[i] if top_urls[i] else "")
        if similarity_scores[i] > threshold:
            n += 1
            info_path = top_ids[i]
            if file_name:
                chunk_file = info_path.split('>')[0].strip()
                if chunk_file in file_name:
                    info_path = '>'.join(info_path.split('>')[1:])
            file_path = top_files[i]
            topic_path = top_topic_paths[i]
            url = top_urls[i] if top_urls[i] else ""
            insert_document += (
                f'Reference Number: {n}\n'
                f"Directory Path to reference file to tell what file is about: {file_path}\n"
                f"Topic Path of chunk in file to tell the topic of chunk: {topic_path}\n"
                f'Document: {top_docs[i]}\n\n'
            )
            reference_list.append([info_path, url, file_path])
    if not audio_response:
        response_style=f"Answer the instruction thoroughly with a well-structured markdown format answer. No references at the end."
        reference_style=f"Refer to specific reference numbers inline using [Reference: n] style. Do not list references at the end. "
    else:
        response_style = """
        STYLE:
        Use a clear, natural, speaker-friendly tone that is short and engaging. No code block or Markdown formatting. One sentence per line. No references at the end.

        FORMATTING:
        After every sentence, insert a newline character (\n).
        Two newline characters (\n\n) indicate a new paragraph.
        Do not join sentences in the same line.
        """
        reference_style = f"Mention specific reference numbers inline when that part of the answer is refer to some reference. Do not mention references at the end of the response as user cannot connect them to the answer. e.g. According to reference 1, as mention in reference 2, etc. "

    if not insert_document or n == 0:
        modified_message = (
            f"{response_style}"
            f"If the question is a complex question, provide hints, explanations, or step-by-step guidance instead of a direct answer. "
            f"If you are unsure after making a reasonable effort, explain that there is no data in the knowledge base for the response. "
            f"Only refuse if the question is clearly unrelated to any topic in {course}: {class_name} and is not a general, reasonable query. "
            f"If the intent is unclear, ask clarifying questions rather than refusing.\n---\n"
        )
    else:
        modified_message = (
            f"{insert_document}---\n"
            f"{response_style}"
            f"Review the reference documents, considering their Directory Path (original file location), Topic Path (section or title it belongs to), and Document content. "
            f"Select only the most relevant references to answer the instruction thoroughly in a well-structured markdown format (do not add '```markdown'). "
            f"Ground your answer in the facts from these selected references. "
            f"If the question is a complex problem, provide hints, explanations, or step-by-step guidance instead of giving the direct answer. "
            f"{reference_style}"
            f"Exclude and avoid explaining irrelevant references. "
            f"If, after reasonable effort, no relevant information is found, state that there is no data in the knowledge base for the response. "
            f"Refuse only if the question is clearly unrelated to any topic in {course}: {class_name}, is not a general query, and has no link to the provided references. "
            f"If intent is unclear, ask clarifying questions before refusing.\n---\n"
        )
    if not practice:
        modified_message += f"Instruction: {user_message}"
    else:
        modified_message += user_message
    return modified_message, reference_list


async def local_parser(
        stream: Any, reference_list: List[str]
) -> Generator[str, None, None]:
    """
    Yield tokens from a text stream and append the reference block at the end.
    TODO: This function can be removed in the future once the legacy code migration is completed.
    """
    print('Response:')
    previous_text = ""
    async for output in stream:
        text = output.outputs[0].text
        chunk = text[len(previous_text):]
        yield chunk
        previous_text = text
        print(chunk, end="")

    pattern = re.compile(
        r'(?:\[Reference:\s*([\d,\s]+)\]|\breferences?\s+(\d+(?:\s*(?:,|and)\s*\d+)*))',
        re.IGNORECASE
    )

    mentioned_references = {
        int(n)
        for m in pattern.finditer(previous_text)
        for n in re.findall(r'\d+', m.group(1) or m.group(2))
    }
    print("\n\nMentioned references:", mentioned_references)

    lines = []
    max_idx = len(reference_list)
    for i in sorted(mentioned_references):
        if 1 <= i <= max_idx:
            info_path, url, file_path = reference_list[i - 1]
            lines.append(
                f"Reference {i}: "
                f"<|begin_of_reference_name|>{i}: {info_path}<|end_of_reference_name|>"
                f"<|begin_of_reference_link|>{url}<|end_of_reference_link|>"
                f"<|begin_of_file_path|>{file_path}<|end_of_file_path|>"
                f"<|begin_of_index|>1<|end_of_index|>"
            )
    if lines:
        reference_string = "\n\n".join(lines)
        ref_block = f"\n\n<|begin_of_reference|>\n\n{reference_string}\n<|end_of_reference|>"
        yield ref_block
        print(ref_block)


async def parse_token_stream_for_json(stream: Any) -> Generator[str, None, None]:
    """
    Yield tokens from a text stream (simplified version for JSON output).
    """
    previous_text = ""
    async for output in stream:
        text = output.outputs[0].text
        chunk = text[len(previous_text):]
        yield chunk
        previous_text = text


def format_chat_msg(messages: List[Message]) -> List[Message]:
    """
    Format a conversation by prepending an initial system message.
    """
    response: List[Message] = []
    system_message = (
        "You are TAI, a helpful AI assistant. Your role is to answer questions or provide guidance to the user. "
        "When responding to complex question that cannnot be answered directly by provided reference material, prefer not to give direct answers. Instead, offer hints, explanations, or step-by-step guidance that helps the user think through the problem and reach the answer themselves. "
        "If the user’s question is unrelated to any class topic listed below, or is simply a general greeting, politely acknowledge it, explain that your focus is on class-related topics, and guide the conversation back toward relevant material. Focus on the response style, format, and reference style."
    )
    response.append(Message(role="system", content=system_message))
    for message in messages:
        response.append(Message(role=message.role, content=message.content))
    return response

import asyncio
async def generate_user_query(messages: List[Any], engine: Any) -> str:
    """
    Reformulate the latest user request into a single self-contained query string,
    based on the full chat history (user + assistant messages).
    Returns plain text with no quotes or extra formatting.
    """
    system_prompt = (
        "You are a query reformulator for a RAG system. "
        "From the full chat history, rewrite the latest user request as a single, "
        "self-contained question for document retrieval. "
        "Resolve pronouns and references using context, include relevant constraints "
        "(dates, versions, scope), and avoid adding facts not in the history. "
        "Return only the rewritten query as question in plain text—no quotes, no extra text."
    )
    chat = [{"role": "system", "content": system_prompt}]
    chat.extend(
        {
            "role": m.role,
            "content": m.content.split("<|begin_of_reference|>", 1)[0].strip(),
        }
        for m in messages
        if m.role in ("user", "assistant")
    )
    prompt = tokenizer.apply_chat_template(
        chat, tokenize=False, add_generation_prompt=True
    )
    text = ""
    async for chunk in engine.generate(
        prompt=prompt,
        sampling_params=SAMPLING,
        request_id=str(time.time_ns())
    ):
        text = chunk.outputs[0].text  # n=1 completion
    return text


async def generate_chat_response(
        messages: List[Message],
        stream: bool = True,
        rag: bool = True,
        course: Optional[str] = None,
        threshold: float = 0.32,
        top_k: int = 7,
        engine: Any = None,
        audio_response: bool = False,
) -> Tuple[Any, List[str| Any]]:
    """
    Build an augmented message with references and run LLM inference.
    Returns a tuple: (stream, reference_string)
    """
    query_message = await generate_user_query(messages, engine) if len(messages) > 2 else messages[-1].content
    user_message = messages[-1].content
    modified_message, reference_list = build_augmented_prompt(
        user_message, course if course else "", threshold, rag, top_k, query_message=query_message, audio_response=audio_response
    )

    messages[-1].content = modified_message
    if is_local_engine(engine):
        iterator = generate_streaming_response(messages, engine)
        return iterator, reference_list
    else:
        response = engine(messages[-1].content, stream=stream, course=course)
        return response, reference_list


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
) -> Tuple[Any, List[str | Any]]:
    """
    Build an augmented message with references and run LLM inference.
    Returns a tuple: (stream, reference_string)
    """
    user_message = messages[-1].content
    modified_message, reference_list = build_augmented_prompt(
        user_message, course if course else "", threshold, rag, top_k, practice=True,
        problem_content=problem_content, answer_content=answer_content, file_name=file_name
    )

    messages[-1].content = modified_message
    if is_local_engine(engine):
        iterator = generate_streaming_response(messages, engine)
        return iterator, reference_list
    else:
        response = engine(messages[-1].content, stream=stream, course=course)
        return response, reference_list


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
