import time
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

from app.core.models.chat_completion import Message, UserFocus
from app.services.generation.message_format import format_chat_msg
from app.services.query.reformulation import build_retrieval_query
from app.services.query.prompt_assembly import build_augmented_prompt, build_prompt_from_refs
from app.services.query.file_context import build_file_augmented_context
from app.services.query.vector_search import get_relevant_file_descriptions, get_two_stage_references
from app.services.request_timer import RequestTimer


@dataclass
class TutorContext:
    """Result of the query step for tutor mode."""
    messages: List[Message]
    reference_list: List[Tuple]


async def build_tutor_context(
    messages: List[Message],
    user_focus: Optional[UserFocus],
    answer_content: Optional[str],
    problem_content: Optional[str],
    course: Optional[str],
    engine: Any,
    sid: Optional[str],
    timer: Optional[RequestTimer],
    audio_response: bool = False,
) -> TutorContext:
    """
    Step 1: Build the complete prompt context for tutor mode.

    Pipeline:
    1. Format messages with tutor mode system prompt
    2. Build file context (if user_focus)
    3. Retrieve memory synopsis (if sid)
    4. Reformulate query
    5. Assemble RAG-augmented prompt
    """
    # 1. Message formatting (tutor_mode=True for tutor)
    messages = format_chat_msg(
        messages,
        tutor_mode=True,
        audio_response=audio_response
    )

    user_message = messages[-1].content
    messages[-1].content = ""

    t0 = time.time()

    # 2. File context (if user_focus)
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
        augmented_context, _, filechat_focused_chunk, filechat_file_sections = build_file_augmented_context(
            file_uuid, selected_text, index)
        messages[-1].content = (
            f"{augmented_context}"
            f"Below are the relevant references for answering the user:\n\n"
        )

    # 3. Memory retrieval
    previous_memory = None
    if sid and len(messages) > 2:
        try:
            from app.services.memory.service import MemorySynopsisService
            memory_service = MemorySynopsisService()
            previous_memory = await memory_service.get_by_chat_history_sid(sid)
        except Exception as e:
            print(f"[INFO] Failed to retrieve memory for query building, continuing without: {e}")
            previous_memory = None

    # 4. Query reformulation
    if timer:
        timer.mark("query_reformulation_start")

    course_descriptions = get_relevant_file_descriptions(user_message, course) if course else None

    query_message = await build_retrieval_query(user_message, previous_memory,
                                                filechat_file_sections, filechat_focused_chunk,
                                                course_descriptions=course_descriptions)

    if not query_message:
        print("[WARNING] Reformulation returned empty query, falling back to original user message")
        query_message = user_message

    if timer:
        timer.mark("query_reformulation_end")

    print(f"[INFO] Preprocessing time: {time.time() - t0:.2f} seconds")

    # 5. Two-stage retrieval + outline prompt assembly
    refs, class_name = get_two_stage_references(
        query_message,
        course if course else "",
        top_k_files=7,
        top_k_chunks_per_file=3,
        threshold=0.32,
        timer=timer,
    )

    modified_message, reference_list, system_add_message = build_prompt_from_refs(
        user_message=user_message,
        course=course if course else "",
        class_name=class_name,
        refs=refs,
        problem_content=problem_content,
        answer_content=answer_content,
        audio_response=audio_response,
        tutor_mode=True,
        outline_mode=True,
    )

    messages[-1].content += modified_message
    messages[0].content = system_add_message

    return TutorContext(messages=messages, reference_list=reference_list)
