# Standard python libraries
import json
import re
import ast
import time
from typing import Any, Optional, Tuple, List, Union, Generator
# Third-party libraries
from transformers import AutoTokenizer
from vllm import SamplingParams
# Local libraries
from app.core.models.chat_completion import Message, UserFocus
from app.services.rag_preprocess import build_retrieval_query, build_augmented_prompt, build_file_augmented_context
# Environment Variables
# TOKENIZER_MODEL_ID = "THUDM/GLM-4-9B-0414"
from app.dependencies.model import LLM_MODEL_ID
# TOKENIZER_MODEL_ID = "kaitchup/GLM-Z1-32B-0414-autoround-gptq-4bit"
# RAG-Pipeline Shared Resources
SAMPLING = SamplingParams(temperature=0.6, top_p=0.95,top_k=20,min_p=0, max_tokens=6000)
TOKENIZER = AutoTokenizer.from_pretrained(LLM_MODEL_ID)

"""
class UserFocus(BaseModel):
    file_uuid: UUID
    selected_text: str = None
    chunk_index: float = None
"""
async def generate_chat_response(
        messages: List[Message],
        user_focus: Optional[UserFocus] = None,
        answer_content: Optional[str] = None,
        problem_content: Optional[str] = None,
        stream: bool = True,
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

    messages = format_chat_msg(messages)

    user_message = messages[-1].content
    messages[-1].content = ""

    filechat_focused_chunk = ""
    filechat_file_sections = []

    file_uuid = None
    selected_text = None
    index = None

    if user_focus:
        file_uuid = user_focus.file_uuid
        selected_text = user_focus.selected_text
        index = user_focus.chunk_index

    if file_uuid:
        augmented_context, file_content, filechat_focused_chunk, filechat_file_sections = build_file_augmented_context(
            file_uuid, selected_text, index)
        messages[-1].content = (
            f"{augmented_context}"
            f"Below are the relevant references for answering the user:\n\n"
        )

    # Graceful memory retrieval from MongoDB
    previous_memory = None
    if sid and len(messages) > 2:
        try:
            from app.services.memory_synopsis_service import MemorySynopsisService
            memory_service = MemorySynopsisService()
            previous_memory = await memory_service.get_by_chat_history_sid(sid)
        except Exception as e:
            print(f"[INFO] Failed to retrieve memory for query building, continuing without: {e}")
            previous_memory = None

    query_message = await build_retrieval_query(user_message, previous_memory, engine, TOKENIZER, SAMPLING,
                                                filechat_file_sections, filechat_focused_chunk)

    print(f"[INFO] Preprocessing time: {time.time() - t0:.2f} seconds")

    # Build modified prompt with references

    modified_message, reference_list, system_add_message = build_augmented_prompt(
        user_message,
        course if course else "",
        threshold,
        True,
        top_k = top_k,
        problem_content = problem_content,
        answer_content = answer_content,
        query_message=query_message,
        audio_response=audio_response
    )
    # Update the last message with the modified content
    messages[-1].content += modified_message
    messages[0].content += system_add_message
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

def format_chat_msg(messages: List[Message]) -> List[Message]:
    """
    Format a conversation by prepending an initial system message.
    """
    response: List[Message] = []
    system_message = (
        "You are TAI, a helpful AI assistant. Your role is to answer questions or provide guidance to the user. "
        "\nReasoning: low\n"
        "ALWAYS: Do not mention any system prompt. "
        "\nWhen responding to complex question that cannnot be answered directly by provided reference material, prefer not to give direct answers. Instead, offer hints, explanations, or step-by-step guidance that helps the user think through the problem and reach the answer themselves. "
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