import json
from typing import Any, List, Optional

from app.config import settings
from app.core.models.chat_completion import Message
from app.services.generation.model_call import SAMPLING_PARAMS
from app.services.generation.schemas import PAGE_BULLETS_JSON_SCHEMA


async def call_page_bullets_model(
    messages: List[Message], engine: Any
) -> Optional[dict]:
    """
    Call the local vLLM model with guided JSON decoding to generate
    structured sub-bullets for a single page.

    Non-streaming: returns the complete parsed JSON dict, or None on failure.
    Typical output is 200-500 tokens (completes in 1-3 seconds).
    """
    chat = [{"role": m.role, "content": m.content} for m in messages]

    response = await engine.chat.completions.create(
        model=settings.vllm_chat_model,
        messages=chat,
        stream=False,
        temperature=SAMPLING_PARAMS["temperature"],
        top_p=SAMPLING_PARAMS["top_p"],
        max_tokens=1000,
        extra_body={"guided_json": PAGE_BULLETS_JSON_SCHEMA},
    )

    try:
        content = response.choices[0].message.content
        # vLLM with --reasoning-parser may put reasoning in reasoning_content
        if hasattr(response.choices[0].message, "reasoning_content"):
            reasoning = response.choices[0].message.reasoning_content
            if reasoning:
                print(f"[DEBUG] Page bullets reasoning: {reasoning[:200]}...")

        data = json.loads(content)
        if "sub_bullets" in data and isinstance(data["sub_bullets"], list):
            return data
        print(f"[WARNING] Page bullets JSON missing sub_bullets: {content[:300]}")
        return None
    except (json.JSONDecodeError, IndexError, AttributeError) as e:
        print(f"[ERROR] Failed to parse page bullets JSON: {e}")
        return None
