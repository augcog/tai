from typing import Any, List

from app.config import settings
from app.core.models.chat_completion import Message
from app.services.generation.model_call import SAMPLING_PARAMS


async def call_page_content_model(messages: List[Message], engine: Any):
    """
    Call the local vLLM model for page content generation.

    No response_format — output is plain markdown, not JSON.
    Yields raw streaming chunks from the vLLM server.
    """
    chat = [{"role": m.role, "content": m.content} for m in messages]

    stream = await engine.chat.completions.create(
        model=settings.vllm_chat_model,
        messages=chat,
        stream=True,
        temperature=SAMPLING_PARAMS["temperature"],
        top_p=SAMPLING_PARAMS["top_p"],
        max_tokens=SAMPLING_PARAMS["max_tokens"],
        extra_body=SAMPLING_PARAMS["extra_body"],
    )

    async for chunk in stream:
        if chunk.choices:
            yield chunk
