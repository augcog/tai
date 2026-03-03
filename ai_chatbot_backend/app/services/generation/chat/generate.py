from typing import Any, List, Optional

from app.core.models.chat_completion import Message
from app.services.generation.model_call import (
    is_openai_client,
    generate_streaming_response,
    call_remote_engine,
)


async def call_chat_model(
    messages: List[Message],
    engine: Any,
    stream: bool = True,
    course: Optional[str] = None,
):
    """
    Step 2: Call LLM for regular chat mode (no JSON schema).

    Returns either a streaming iterator or a complete response string.
    """
    if is_openai_client(engine):
        return generate_streaming_response(messages, engine)

    # Remote engine: no response_format for regular chat
    return await call_remote_engine(
        messages, engine, stream=stream, course=course, response_format=None
    )
