# Standard python libraries
import json
from dataclasses import asdict, dataclass, field
from typing import Any, List, Optional

# Third-party libraries
from openai import AsyncOpenAI, OpenAI

from app.config import settings
# Local libraries
from app.core.models.chat_completion import Message
from app.services.memory import prompts as memory_prompts
from app.services.generation.schemas import MEMORY_SYNOPSIS_JSON_SCHEMA


@dataclass
class MemorySynopsis:
    """
    Compact, structured memory about a conversation. Keep it small & stable.
    """
    focus: str = ""                                             # What is the main topic / intent of the user?
    user_goals: List[str] = field(default_factory=list)         # User's explicit goals/preferences
    constraints: List[str] = field(default_factory=list)        # Hard constraints (versions, dates, scope, etc.)
    key_entities: List[str] = field(default_factory=list)       # People, products, datasets, repos, courses…
    artifacts: List[str] = field(default_factory=list)          # Files, URLs, IDs, paths mentioned
    open_questions: List[str] = field(default_factory=list)     # Unresolved questions the user asked
    action_items: List[str] = field(default_factory=list)       # TODOs, "next steps"
    decisions: List[str] = field(default_factory=list)          # Agreed choices so far

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False)

    @staticmethod
    def from_json(s: str) -> "MemorySynopsis":
        data = json.loads(s or "{}")
        return MemorySynopsis(**{k: data.get(k, v) for k, v in asdict(MemorySynopsis()).items()})


async def build_memory_synopsis(
        messages: List[Message],
        engine: Any,
        prev_synopsis: Optional[MemorySynopsis] = None,
        chat_history_sid: Optional[str] = None,
        max_prompt_tokens: int = 3500,
) -> MemorySynopsis:
    """
    Create/refresh the rolling MemorySynopsis.
    - messages: full chat so far (system/assistant/user)
    - prev_synopsis: prior memory to carry forward (we'll merge)
    - chat_history_sid: if provided, retrieves previous memory from MongoDB
    """
    # Graceful MongoDB retrieval for previous memory
    if chat_history_sid and not prev_synopsis:
        try:
            from app.services.memory.service import MemorySynopsisService
            memory_service = MemorySynopsisService()
            prev_synopsis = await memory_service.get_by_chat_history_sid(chat_history_sid)
        except Exception as e:
            print(f"[INFO] Failed to retrieve previous memory, generating from scratch: {e}")
            prev_synopsis = None  # Continue without previous memory

    transcript = _render_transcript(messages)
    cur = await _llm_synopsis_from_transcript(engine, transcript, max_prompt_tokens=max_prompt_tokens)
    if prev_synopsis:
        cur = await _llm_merge_synopses(engine, prev_synopsis, cur)

    # tighten fields (keep stable, short)
    cur.focus = _truncate_sentence(cur.focus, 180)
    cur.user_goals = cur.user_goals[:8]
    cur.constraints = cur.constraints[:8]
    cur.key_entities = cur.key_entities[:16]
    cur.artifacts = cur.artifacts[:16]
    cur.open_questions = cur.open_questions[:8]
    cur.action_items = cur.action_items[:8]
    cur.decisions = cur.decisions[:8]
    return cur


# Memory synopsis prompts
_LLM_SYSTEM = memory_prompts.SYNOPSIS_SYSTEM
_LLM_USER_TEMPLATE = memory_prompts.SYNOPSIS_USER_TEMPLATE


async def _llm_synopsis_from_transcript(
        engine: Any,
        transcript: str,
        max_prompt_tokens: int = 3500,
) -> MemorySynopsis:
    """
    Use vLLM server to compress the transcript into MemorySynopsis JSON.
    """
    # Check if engine is OpenAI client
    if not isinstance(engine, (OpenAI, AsyncOpenAI)):
        # Fallback for non-OpenAI engines
        return MemorySynopsis()

    # Truncate transcript if needed (rough estimate: 1 token ~= 4 chars)
    max_chars = max_prompt_tokens * 4
    if len(transcript) > max_chars:
        transcript = transcript[-max_chars:]

    # Prepare the system and user messages for the LLM
    sys_msg = {"role": "system", "content": _LLM_SYSTEM}
    usr = {
        "role": "user",
        "content": _LLM_USER_TEMPLATE.format(transcript=transcript)
    }
    chat = [sys_msg, usr]

    # Generate the synopsis using the OpenAI API with JSON mode
    response = await engine.chat.completions.create(
        model=settings.vllm_chat_model,
        messages=chat,
        temperature=0.0,
        top_p=1.0,
        max_tokens=800,
        response_format={"type": "json_object"},
        extra_body={"guided_json": MEMORY_SYNOPSIS_JSON_SCHEMA}
    )

    # vLLM with --reasoning-parser separates reasoning_content from content
    # Use content directly (final response without thinking)
    text = response.choices[0].message.content or "{}"
    try:
        print('Generated MemorySynopsis JSON:', json.dumps(json.loads(text), indent=2, ensure_ascii=False))
    except (ValueError, TypeError):
        print('Generated MemorySynopsis JSON:', text)
    try:
        data = json.loads(text.strip())
    except json.JSONDecodeError:
        print('Failed to parse merged MemorySynopsis JSON:', text)
        return MemorySynopsis()
    return MemorySynopsis(**data)


def _render_transcript(messages: List[Message], max_chars: int = 12000) -> str:
    """
    Linearized transcript with role tags. Keep it simple for robustness.
    """
    lines: List[str] = []
    for m in messages:
        role = (getattr(m, "role", None) or "user").lower()
        content = getattr(m, "content", "").strip()
        lines.append(f"{role.capitalize()}: {content}")
    text = "\n".join(lines)
    return text if len(text) <= max_chars else text[-max_chars:]


def _truncate_sentence(s: str, max_chars: int) -> str:
    return s if len(s) <= max_chars else s[:max_chars - 1] + "…"


# Memory merge prompts
_LLM_MERGE_SYSTEM = memory_prompts.MERGE_SYSTEM
_LLM_MERGE_USER_TEMPLATE = memory_prompts.MERGE_USER_TEMPLATE


async def _llm_merge_synopses(
    engine: Any,
    old: MemorySynopsis,
    new: MemorySynopsis,
) -> MemorySynopsis:
    # Check if engine is OpenAI client
    if not isinstance(engine, (OpenAI, AsyncOpenAI)):
        # Fallback: just return new synopsis
        return new

    old_json = MemorySynopsis(**asdict(old)).to_json()
    new_json = MemorySynopsis(**asdict(new)).to_json()

    # Prepare the system and user messages for the LLM
    sys_msg = {"role": "system", "content": _LLM_MERGE_SYSTEM}
    usr_msg = {"role": "user", "content": _LLM_MERGE_USER_TEMPLATE.format(old_json=old_json, new_json=new_json)}
    chat = [sys_msg, usr_msg]

    # Generate the merged synopsis using the OpenAI API
    response = await engine.chat.completions.create(
        model=settings.vllm_chat_model,
        messages=chat,
        temperature=0.0,
        top_p=1.0,
        max_tokens=800,
        response_format={"type": "json_object"},
        extra_body={"guided_json": MEMORY_SYNOPSIS_JSON_SCHEMA}
    )

    # vLLM with --reasoning-parser separates reasoning_content from content
    # Use content directly (final response without thinking)
    text = response.choices[0].message.content or "{}"
    # try to parse JSON, if fails return empty MemorySynopsis
    try:
        data = json.loads(text.strip())
    except json.JSONDecodeError:
        print('Failed to parse merged MemorySynopsis JSON:', text)
        return MemorySynopsis()
    return MemorySynopsis(**data)
