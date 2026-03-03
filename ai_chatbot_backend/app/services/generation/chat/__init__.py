from typing import Any, List, Optional

from app.core.models.chat_completion import Message, UserFocus
from app.services.request_timer import RequestTimer


async def run_chat_pipeline(
    messages: List[Message],
    user_focus: Optional[UserFocus] = None,
    answer_content: Optional[str] = None,
    problem_content: Optional[str] = None,
    stream: bool = True,
    course: Optional[str] = None,
    engine: Any = None,
    audio_response: bool = False,
    sid: Optional[str] = None,
    timer: Optional[RequestTimer] = None,
    audio_text: Optional[str] = None,
):
    """
    Regular chat pipeline: query -> generate -> handler.

    Handles TEXT_CHAT_REGULAR and VOICE_REGULAR modes.
    Returns an async generator (streaming) or a response string (non-streaming).
    """
    from .query import build_chat_context
    from .generate import call_chat_model

    # Step 1: Query — build context
    context = await build_chat_context(
        messages, user_focus, answer_content, problem_content,
        course, engine, sid, timer,
        audio_response=audio_response,
    )

    # Step 2: Generate — call LLM
    if not stream:
        response = await call_chat_model(
            context.messages, engine, stream=False, course=course
        )
        return response

    raw_stream = await call_chat_model(
        context.messages, engine, stream=True, course=course
    )

    # Step 3: Handler — process streaming output
    from .handler import ChatHandler
    handler = ChatHandler(
        stream=raw_stream,
        reference_list=context.reference_list,
        audio_response=audio_response,
        course_code=course,
        audio_text=audio_text,
        timer=timer,
    )
    return handler.run()
