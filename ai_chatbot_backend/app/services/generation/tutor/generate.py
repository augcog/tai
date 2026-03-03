from typing import Any, List, Optional

from app.core.models.chat_completion import Message
from app.services.generation.model_call import (
    is_openai_client,
    generate_streaming_response,
    call_remote_engine,
)
from app.services.generation.schemas import (
    RESPONSE_BLOCKS_OPENAI_FORMAT,
    VOICE_TUTOR_OPENAI_FORMAT,
    OUTLINE_OPENAI_FORMAT,
)

# TODO： add a new function to separate the voice explanation and text explanation.
async def call_tutor_model(
    messages: List[Message],
    engine: Any,
    stream: bool = True,
    audio_response: bool = False,
    course: Optional[str] = None,
    outline_mode: bool = False,
):
    """
    Step 2: Call LLM for tutor mode (with JSON schema).

    Selects the appropriate JSON schema:
    - Outline mode: OUTLINE_OPENAI_FORMAT
    - Voice tutor: VOICE_TUTOR_OPENAI_FORMAT
    - Text tutor: RESPONSE_BLOCKS_OPENAI_FORMAT

    Returns either a streaming iterator or a complete response string.
    """
    if is_openai_client(engine):
        return generate_streaming_response(messages, engine)

    # Debug: print full prompt before sending to model
    print("\n" + "=" * 60)
    print("[DEBUG] Full prompt sent to tutor model:")
    print("=" * 60)
    for msg in messages:
        role = msg.role if hasattr(msg, 'role') else 'unknown'
        content = msg.content if hasattr(msg, 'content') else str(msg)
        print(f"\n--- [{role}] ({len(content)} chars) ---")
        print(content)
    print("=" * 60 + "\n")

    # Remote engine: select JSON schema
    if outline_mode:
        response_format = OUTLINE_OPENAI_FORMAT
    elif audio_response:
        response_format = VOICE_TUTOR_OPENAI_FORMAT


    return await call_remote_engine(
        messages, engine, stream=stream, course=course, response_format=response_format
    )
