# Standard python libraries
import re
import ast
import time
from typing import Any, Optional, Tuple, List, Union, Generator
from uuid import UUID
# Third-party libraries
from transformers import AutoTokenizer
from vllm import SamplingParams
# Local libraries
from app.core.models.chat_completion import Message
from app.services.rag_preprocess import build_retrieval_query, build_augmented_prompt, build_file_augmented_context
from app.services.rag_postprocess import build_memory_synopsis
# Environment Variables
TOKENIZER_MODEL_ID = "THUDM/GLM-4-9B-0414"
# RAG-Pipeline Shared Resources
SAMPLING = SamplingParams(temperature=0.3, top_p=0.95, max_tokens=4096)
TOKENIZER = AutoTokenizer.from_pretrained(TOKENIZER_MODEL_ID)
LOCAL_MEMORY_SYNOPSIS = {}


async def generate_chat_response(
        messages: List[Message],
        stream: bool = True,
        rag: bool = True,
        course: Optional[str] = None,
        threshold: float = 0.32,
        top_k: int = 7,
        engine: Any = None,
        audio_response: bool = False,
        sid: Optional[str] = None
) -> Tuple[Any, List[str | Any]]:
    """
    Build an augmented message with references and run LLM inference.
    Returns a tuple: (stream, reference_string)
    """
    # Build the query message based on the chat history
    t0 = time.time()
    user_message = messages[-1].content
    query_message = await build_retrieval_query(user_message, LOCAL_MEMORY_SYNOPSIS.get(sid, None), engine, TOKENIZER, SAMPLING) if len(messages) > 2 else user_message
    print(f"[INFO] Preprocessing time: {time.time() - t0:.2f} seconds")

    # Build modified prompt with references
    modified_message, reference_list = build_augmented_prompt(
        user_message,
        course if course else "",
        threshold,
        rag,
        top_k=top_k,
        query_message=query_message,
        audio_response=audio_response
    )
    # Update the last message with the modified content
    messages[-1].content = modified_message
    # Generate the response using the engine
    if _is_local_engine(engine):
        iterator = _generate_streaming_response(messages, engine)
        return iterator, reference_list
    else:
        response = engine(messages[-1].content, stream=stream, course=course)
        return response, reference_list


async def generate_file_chat_response(
        messages: List[Message],
        file_uuid: UUID,
        # document: str,
        selected_text: Optional[str] = None,
        index: Optional[float] = None,
        stream: bool = True,
        rag: bool = True,
        course: Optional[str] = "",
        threshold: float = 0.32,
        top_k: int = 7,
        engine: Any = None,
        audio_response: bool = False,   # Not supported yet
        sid: Optional[str] = None
) -> Tuple[Any, str]:
    """
    Build an augmented message with references and run LLM inference for file-based chat.
    Returns a tuple: (stream, reference_string)
    """
    # Build the augmented context for file-based chat
    augmented_context, reference_list = build_file_augmented_context(
        file_uuid, course, threshold, rag, selected_text, index, top_k
    )

    t0 = time.time()
    user_message = messages[-1].content
    query_message = await build_retrieval_query(user_message, LOCAL_MEMORY_SYNOPSIS.get(sid, None), engine, TOKENIZER, SAMPLING) if len(messages) > 2 else user_message
    print(f"[INFO] Preprocessing time: {time.time() - t0:.2f} seconds")
    modified_message, reference_list = build_augmented_prompt(
        user_message,
        course,
        threshold,
        rag,
        top_k=top_k,
        query_message=query_message,
        reference_list=reference_list
    )

    messages[-1].content = (
        f"{augmented_context}"
        f"Besides the context of the file and related references above, you also have the following references based on the chat history:\n\n"
        f"{modified_message}"
    )

    # Generate the response using the engine
    if _is_local_engine(engine):
        iterator = _generate_streaming_response(messages, engine)
        return iterator, reference_list
    else:
        response = engine(messages[-1].content, stream=stream, course=course)
        return response, reference_list


def _is_local_engine(engine: Any) -> bool:
    """
    Check if the engine is a local instance by verifying if it has an 'is_running' attribute.
    """
    return hasattr(engine, "is_running") and engine.is_running


def _generate_streaming_response(messages: List[Message], engine: Any = None) -> Any:
    """
    Generate a streaming response from the model based on the provided messages.
    """
    chat = [
        {"role": m.role, "content": m.content, "tool_call_id": m.tool_call_id}
        for m in messages
    ]
    prompt = TOKENIZER.apply_chat_template(chat, tokenize=False, add_generation_prompt=True)
    return engine.generate(prompt, SAMPLING, request_id=str(time.time_ns()))


async def local_parser(
        stream: Any,
        reference_list: List[str],
        messages: Optional[List[Message]] = None,
        engine: Optional[Any] = None,
        old_sid: Optional[str] = None
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
    # Extract mentioned references from the previous text
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
    # Generate the reference block
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

async def build_memory_after_response(
        messages: List[Message],
        previous_text: str,
        references: List[Any],
        engine: Optional[Any] = None,
        old_sid: Optional[str] = None
) -> Any:
    # Call to build memory after finish output.
    if messages and engine and old_sid:
        messages.append(Message(role="assistant", content=previous_text + "\n<!--REFERENCES:" + "\n\n".join([str(ref) for ref in references]) + "-->"))
        sid = sid_from_history(messages)
        print(f"[INFO] Generated SID: {sid}")
        LOCAL_MEMORY_SYNOPSIS[sid] = await build_memory_synopsis(messages, TOKENIZER, engine, LOCAL_MEMORY_SYNOPSIS.get(old_sid, None))
        if old_sid in LOCAL_MEMORY_SYNOPSIS:
            LOCAL_MEMORY_SYNOPSIS.pop(old_sid, None)
            print(f"[INFO] Removed SID: {old_sid}")
        print(f"\n\n---LOCAL_MEMORY_SYNOPSIS---\n\n{LOCAL_MEMORY_SYNOPSIS[sid]}\n\n")


import json, base64, hashlib, secrets
def sid_from_history(messages):
    hist = [{"r": getattr(m, "role", "user"),
             "c": (getattr(m, "content", "") or "").split("\n<!--REFERENCES:", 1)[0]}
            for m in messages]
    # print(f"[INFO] History for SID generation: {hist}")
    digest = hashlib.blake2b(
        json.dumps(hist, separators=(",", ":"), ensure_ascii=False).encode(),
        digest_size=12
    ).digest()
    return base64.urlsafe_b64encode(digest).decode().rstrip("=")


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


#############################################################################
########################### UNKNOWN USAGE YET ###############################
#############################################################################


def _to_str_list(x: Union[str, List], *, trim=True) -> List[str]:
    if isinstance(x, list):
        items = x
    elif isinstance(x, str):
        try:
            parsed = json.loads(x)
            items = parsed if isinstance(parsed, list) else [str(parsed)]
        except Exception:
            try:
                parsed = ast.literal_eval(x)
                items = parsed if isinstance(parsed, list) else [str(parsed)]
            except Exception:
                parts = re.findall(r'"([^"]+)"|\'([^\']+)\'', x)
                items = [a or b for a, b in parts]
    else:
        items = [str(x)]

    items = ["" if i is None else str(i) for i in items]
    if trim:
        items = [i.strip() for i in items]
    return [i for i in items if i != ""]


def join_titles(info_path: Union[str, List], *, sep=" > ", start=0) -> str:
    items = _to_str_list(info_path)
    items = items[start:]
    return sep.join(items)


#############################################################################
############################# LEGACY OLD CODE ###############################
#############################################################################
"""
LEGACY: unknown current usage; remove in future updates.
"""


def generate_practice_response(
        messages: List[Message],
        problem_content: str,
        answer_content: str,
        file_name: str,
        stream: bool = True,
        rag: bool = True,
        course: Optional[str] = None,
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
        user_message, course if course else "", threshold, rag, top_k=top_k, practice=True,
        problem_content=problem_content, answer_content=answer_content
    )

    messages[-1].content = modified_message
    if _is_local_engine(engine):
        iterator = _generate_streaming_response(messages, engine)
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
        user_message,
        course if course else "",
        embedding_dir,
        threshold,
        rag,
        top_k=top_k
    )
    messages[-1].content = modified_message
    if _is_local_engine(engine):
        iterator = _generate_streaming_response(messages, engine)

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