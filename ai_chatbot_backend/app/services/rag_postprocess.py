# Standard python libraries
import time
import json
from dataclasses import dataclass, asdict, field
from typing import Any, List, Optional
# Third-party libraries
from vllm import SamplingParams
from vllm.sampling_params import GuidedDecodingParams
# Local libraries
from app.core.models.chat_completion import Message
import re

def extract_channels(text: str) -> dict:
    if "</think>" in text:
        parts = text.split("</think>", 1)
        return {
            "analysis": parts[0].strip(),
            "final": parts[1].strip()
        }

    incomplete_patterns = ["</think", "</", "<"]
    cleaned_text = text
    for pattern in incomplete_patterns:
        if text.endswith(pattern):
            cleaned_text = text[:-len(pattern)]
            break

    return {
        "analysis": cleaned_text.strip(),
        "final": ""
    }

# Environment variables
MEMORY_SYNOPSIS_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "focus": {"type": "string"},
        "user_goals": {"type": "array", "items": {"type": "string"}},
        "constraints": {"type": "array", "items": {"type": "string"}},
        "key_entities": {"type": "array", "items": {"type": "string"}},
        "artifacts": {"type": "array", "items": {"type": "string"}},
        "open_questions": {"type": "array", "items": {"type": "string"}},
        "action_items": {"type": "array", "items": {"type": "string"}},
        "decisions": {"type": "array", "items": {"type": "string"}},
    },
    "required": ["focus", "user_goals", "constraints", "key_entities",
                 "artifacts", "open_questions", "action_items", "decisions"],
    "additionalProperties": False
}
GUIDED = GuidedDecodingParams(json=MEMORY_SYNOPSIS_JSON_SCHEMA)
SAMPLING_JSON = SamplingParams(
    temperature=0.0, top_p=1.0, max_tokens=800, guided_decoding=GUIDED, skip_special_tokens=False
)


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
    action_items: List[str] = field(default_factory=list)       # TODOs, “next steps”
    decisions: List[str] = field(default_factory=list)          # Agreed choices so far

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False)

    @staticmethod
    def from_json(s: str) -> "MemorySynopsis":
        data = json.loads(s or "{}")
        return MemorySynopsis(**{k: data.get(k, v) for k, v in asdict(MemorySynopsis()).items()})


async def build_memory_synopsis(
        messages: List[Message],
        tokenizer: Any,
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
            from app.services.memory_synopsis_service import MemorySynopsisService
            memory_service = MemorySynopsisService()
            prev_synopsis = await memory_service.get_by_chat_history_sid(chat_history_sid)
        except Exception as e:
            print(f"[INFO] Failed to retrieve previous memory, generating from scratch: {e}")
            prev_synopsis = None  # Continue without previous memory

    transcript = _render_transcript(messages)
    cur = await _llm_synopsis_from_transcript(engine, tokenizer, transcript, max_prompt_tokens=max_prompt_tokens)
    if prev_synopsis:
        cur = await _llm_merge_synopses(engine, tokenizer, prev_synopsis, cur)

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


_LLM_SYSTEM = (
    "You are a memory-synopsis compressor. "
    "Given a chat transcript, produce a STRUCTURED JSON with keys: "
    "focus (string), user_goals (list[str]), constraints (list[str]), key_entities (list[str]), "
    "artifacts (list[str]), open_questions (list[str]), action_items (list[str]), decisions (list[str]). "
    "\nHere's some description of each key: \n"
    "focus - what is the main topic / intent of the user? \n"
    "user_goals - user's explicit goals/preferences. \n"
    "constraints - hard constraints (versions, dates, scope, etc.). \n"
    "key_entities - people, products, datasets, repos, courses… \n"
    "artifacts - files, URLs, IDs, paths mentioned. \n"
    "open_questions - unresolved questions the user asked. \n"
    "action_items - TODOs, “next steps”. \n"
    "decisions - agreed choices so far. \n"
    "\nRules:\n"
    "- Return ONLY a single JSON object that matches the schema keys and types above.\n"
    "- Keep text terse and factual. No markdown, no code fences, no extra commentary.\n"
    "- Arrays must contain strings only; deduplicate items; remove empty strings.\n"
    "- Extract explicit constraints (versions, dates, scope limits) as strings.\n"
)

_LLM_USER_TEMPLATE = """Transcript:
{transcript}

Requirements:
- Summarize tersely.
- Deduplicate entities and URLs/paths.
- Extract explicit constraints (versions, dates, scope limits).
Return ONLY JSON.
"""


async def _llm_synopsis_from_transcript(
        engine: Any,
        tokenizer: Any,
        transcript: str,
        max_prompt_tokens: int = 3500,
) -> MemorySynopsis:
    """
    Use local engine to compress the transcript into MemorySynopsis JSON.
    """
    # Prepare the system and user messages for the LLM
    sys_msg = {"role": "system", "content": _LLM_SYSTEM}
    usr = {
        "role": "user",
        "content": _LLM_USER_TEMPLATE.format(
            transcript=_truncate_to_tokens(tokenizer, transcript, max_prompt_tokens)
        )
    }
    chat = [sys_msg, usr]
    prompt = tokenizer.apply_chat_template(chat, tokenize=False, add_generation_prompt=True)
    # Generate the synopsis using the engine
    text = ""
    async for chunk in engine.generate(
            prompt=prompt,
            sampling_params=SAMPLING_JSON,
            request_id=str(time.time_ns())
    ):
        text = chunk.outputs[0].text
    text = extract_channels(text).get('final', "{}")
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


def _safe_token_len(tokenizer: Any, text: str) -> int:
    """
    Safely estimate the number of tokens in a text using the tokenizer.
    """
    return len(tokenizer.encode(text, add_special_tokens=False))


def _truncate_to_tokens(tokenizer: Any, text: str, max_tokens: int) -> str:
    """
    Truncate text to fit within the max_tokens limit using binary search.
    """
    if _safe_token_len(tokenizer, text) <= max_tokens:
        return text
    # binary chop by characters (fast & simple)
    lo, hi = 0, len(text)
    while lo < hi:
        mid = (lo + hi) // 2
        if _safe_token_len(tokenizer, text[:mid]) <= max_tokens:
            lo = mid + 1
        else:
            hi = mid
    return text[:max(0, lo - 1)]


def _truncate_sentence(s: str, max_chars: int) -> str:
    return s if len(s) <= max_chars else s[:max_chars - 1] + "…"


_LLM_MERGE_SYSTEM = (
    "You merge two conversation memory synopses into ONE, preserving correctness and recency.\n"
    "Output ONLY a JSON object with exactly these keys and types:\n"
    "focus (string), user_goals (list[str]), constraints (list[str]), key_entities (list[str]),\n"
    "artifacts (list[str]), open_questions (list[str]), action_items (list[str]), decisions (list[str]).\n"
    "Rules:\n"
    "- Prefer NEW facts if they add specificity, dates/versions/IDs, or fix errors.\n"
    "- Keep stable facts from OLD if NEW is generic or contradictory.\n"
    "- Deduplicate items; remove empties; keep terse phrasing.\n"
    "- Enforce keeping the most specific and recent at the front of lists.\n"
    "- Do NOT invent facts that are not present in OLD or NEW.\n"
    "- Return ONLY JSON. No markdown, no commentary."
)

_LLM_MERGE_USER_TEMPLATE = """OLD_SYNOPSIS:
{old_json}

NEW_SYNOPSIS:
{new_json}

Task:
Produce the single best merged synopsis following the rules. Return ONLY JSON.
"""


async def _llm_merge_synopses(
    engine: Any,
    tokenizer: Any,
    old: MemorySynopsis,
    new: MemorySynopsis,
) -> MemorySynopsis:
    old_json = MemorySynopsis(**asdict(old)).to_json()
    new_json = MemorySynopsis(**asdict(new)).to_json()
    # Prepare the system and user messages for the LLM
    sys_msg = {"role": "system", "content": _LLM_MERGE_SYSTEM}
    usr_msg = {"role": "user", "content": _LLM_MERGE_USER_TEMPLATE.format(old_json=old_json, new_json=new_json)}
    prompt = tokenizer.apply_chat_template([sys_msg, usr_msg], tokenize=False, add_generation_prompt=True)
    # Generate the merged synopsis using the engine
    text = ""
    async for chunk in engine.generate(
            prompt=prompt,
            sampling_params=SAMPLING_JSON,
            request_id=str(time.time_ns())
    ):
        text = chunk.outputs[0].text
    text = extract_channels(text).get('final', "{}")
    # try to parse JSON, if fails return empty MemorySynopsis
    try:
        data = json.loads(text.strip())
    except json.JSONDecodeError:
        print('Failed to parse merged MemorySynopsis JSON:', text)
        return MemorySynopsis()
    return MemorySynopsis(**data)

