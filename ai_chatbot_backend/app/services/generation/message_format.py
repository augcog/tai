from typing import List

from app.core.models.chat_completion import Message
from app.services.generation.prompts import get_system_prompt


def format_chat_msg(
    messages: List[Message],
    tutor_mode: bool = True,
    audio_response: bool = False
) -> List[Message]:
    """
    Format a conversation by prepending an initial system message based on the 4-mode system.

    4-Mode System:
    - Chat Tutor (tutor_mode=True, audio_response=False): JSON output with tutor guidance
    - Chat Regular (tutor_mode=False, audio_response=False): Plain Markdown, direct answers
    - Voice Tutor (tutor_mode=True, audio_response=True): JSON with unreadable property
    - Voice Regular (tutor_mode=False, audio_response=True): Plain speakable text
    """
    response: List[Message] = []

    system_message = get_system_prompt(tutor_mode=tutor_mode, audio_response=audio_response)

    response.append(Message(role="system", content=system_message))
    for message in messages:
        response.append(Message(role=message.role, content=message.content))
    return response
