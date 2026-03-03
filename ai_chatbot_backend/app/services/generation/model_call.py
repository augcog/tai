# Standard python libraries
from dataclasses import dataclass
from typing import Any, List, Optional

# Third-party libraries
from openai import AsyncOpenAI, OpenAI

from app.config import settings
from app.core.models.chat_completion import Message

# Sampling parameters for generation (used with OpenAI API)
SAMPLING_PARAMS = {
    "temperature": 0.6,
    "top_p": 0.95,
    "max_tokens": 6000,
    "extra_body": {"top_k": 20, "min_p": 0}
}


def resolve_engine(llm_mode: str, openai_model: str) -> Any:
    """Dynamically get an engine based on llm_mode."""
    from app.dependencies.model import get_vllm_chat_client
    from app.dependencies.openai_model import OpenAIModelClient
    if llm_mode == "local":
        return get_vllm_chat_client()
    elif llm_mode == "openai":
        return OpenAIModelClient(
            api_key=settings.openai_api_key,
            model=openai_model
        )
    else:
        raise ValueError(f"Unknown llm_mode: {llm_mode}")


def is_openai_client(engine: Any) -> bool:
    """Check if the engine is an OpenAI or AsyncOpenAI client instance."""
    return isinstance(engine, (OpenAI, AsyncOpenAI))


@dataclass
class MockVLLMOutput:
    """Mock output structure to match vLLM format for OpenAI responses."""
    text: str


@dataclass
class MockVLLMChunk:
    """Mock chunk structure to match vLLM format for OpenAI responses."""
    outputs: List[MockVLLMOutput]


async def generate_streaming_response(messages: List[Message], client: Any):
    """
    Generate a streaming response from the vLLM server using OpenAI chat completions API.

    Yields raw streaming chunks that contain:
    - delta.reasoning_content: The reasoning/thinking content (analysis channel)
    - delta.content: The final response content (final channel)

    The vLLM server with --reasoning-parser flag separates these automatically.
    """
    chat = [
        {"role": m.role, "content": m.content}
        for m in messages
    ]

    stream = await client.chat.completions.create(
        model=settings.vllm_chat_model,
        messages=chat,
        stream=True,
        temperature=SAMPLING_PARAMS["temperature"],
        top_p=SAMPLING_PARAMS["top_p"],
        max_tokens=SAMPLING_PARAMS["max_tokens"],
        extra_body=SAMPLING_PARAMS["extra_body"]
    )

    # Yield raw chunks - handler processes reasoning_content vs content
    async for chunk in stream:
        if chunk.choices:
            yield chunk


async def call_remote_engine(
    messages: List[Message],
    engine: Any,
    stream: bool = True,
    course: Optional[str] = None,
    response_format: Optional[Any] = None,
):
    """
    Shared remote engine call logic for non-OpenAI engines.

    Used by both chat/generate.py and tutor/generate.py to avoid duplication.
    """
    from app.dependencies.openai_model import OpenAIModelClient

    if isinstance(engine, OpenAIModelClient):
        remote_messages = [
            {"role": m.role, "content": m.content}
            for m in messages
        ]
    else:
        remote_messages = [
            {"role": messages[0].role, "content": messages[0].content},
            {"role": messages[-1].role, "content": messages[-1].content},
        ]

    return await engine(
        messages[-1].content,
        messages=remote_messages,
        stream=stream,
        course=course,
        response_format=response_format,
        temperature=SAMPLING_PARAMS["temperature"],
        max_tokens=SAMPLING_PARAMS["max_tokens"],
    )
