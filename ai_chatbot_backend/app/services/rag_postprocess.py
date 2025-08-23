# Standard python libraries
import re
import json
from dataclasses import dataclass, asdict, field
from typing import Any, List, Optional, Iterable
# Third-party libraries
from vllm import SamplingParams
from vllm.sampling_params import GuidedDecodingParams
# Local libraries
from app.core.models.chat_completion import Message

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
        "query_seed": {"type": "string"}
    },
    "required": ["focus", "user_goals", "constraints", "key_entities", "artifacts",
                 "open_questions", "action_items", "decisions", "query_seed"],
    "additionalProperties": False
}
GUIDED = GuidedDecodingParams(json=MEMORY_SYNOPSIS_JSON_SCHEMA)
SAMPLING_JSON = SamplingParams(
    temperature=0.0, top_p=1.0, max_tokens=800, guided_decoding=GUIDED
)


@dataclass
class MemorySynopsis:
    """
    Compact, structured memory about a conversation. Keep it small & stable.
    """
    focus: str = ""  # What is the main topic / intent of the user?
    user_goals: List[str] = field(default_factory=list)  # User's explicit goals/preferences
    constraints: List[str] = field(default_factory=list)  # Hard constraints (versions, dates, scope, etc.)
    key_entities: List[str] = field(default_factory=list)  # People, products, datasets, repos, courses…
    artifacts: List[str] = field(default_factory=list)  # Files, URLs, IDs, paths mentioned
    open_questions: List[str] = field(default_factory=list)  # Unresolved questions the user asked
    action_items: List[str] = field(default_factory=list)  # TODOs, “next steps”
    decisions: List[str] = field(default_factory=list)  # Agreed choices so far
    query_seed: str = ""  # Ultra-short retriever seed for query reformulation

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False)

    @staticmethod
    def from_json(s: str) -> "MemorySynopsis":
        data = json.loads(s or "{}")
        return MemorySynopsis(**{k: data.get(k, v) for k, v in asdict(MemorySynopsis()).items()})


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


_LLM_SYSTEM = (
    "You are a memory-synopsis compressor. "
    "Given a chat transcript, produce a STRUCTURED JSON with keys: "
    "focus (string), user_goals (list[str]), constraints (list[str]), key_entities (list[str]), "
    "artifacts (list[str]), open_questions (list[str]), action_items (list[str]), "
    "decisions (list[str]), query_seed (string). "
    "\nHere's some description of each key: \n"
    "focus - what is the main topic / intent of the user? \n"
    "user_goals - user's explicit goals/preferences. \n"
    "constraints - hard constraints (versions, dates, scope, etc.). \n"
    "key_entities - people, products, datasets, repos, courses… \n"
    "artifacts - files, URLs, IDs, paths mentioned. \n"
    "open_questions - unresolved questions the user asked. \n"
    "action_items - TODOs, “next steps”. \n"
    "decisions - agreed choices so far. \n"
    "query_seed - ultra-short retriever seed for query reformulation. \n"
    "\nRules:\n"
    "- Return ONLY a single JSON object that matches the schema keys and types above.\n"
    "- Keep text terse and factual. No markdown, no code fences, no extra commentary.\n"
    "- Arrays must contain strings only; deduplicate items; remove empty strings.\n"
    "- Extract explicit constraints (versions, dates, scope limits) as strings.\n"
    "- `query_seed` ≤ 300 chars capturing user intent + key constraints, suitable for retrieval.\n"
)

_LLM_USER_TEMPLATE = """Transcript:
{transcript}

Requirements:
- Summarize tersely.
- Deduplicate entities and URLs/paths.
- Extract explicit constraints (versions, dates, scope limits).
- “query_seed” should be <= 300 chars capturing user intent + constraints for retrieval.
Return ONLY JSON.
"""


async def llm_synopsis_from_transcript(
        engine: Any,
        tokenizer: Any,
        transcript: str,
        max_prompt_tokens: int = 3500,
) -> MemorySynopsis:
    """
    Use local engine to compress the transcript into MemorySynopsis JSON.
    """
    from app.services.rag_generation import TOKENIZER
    # Prepare the system and user messages for the LLM
    sys_msg = {"role": "system", "content": _LLM_SYSTEM}
    usr = {
        "role": "user",
        "content": _LLM_USER_TEMPLATE.format(
            transcript=_truncate_to_tokens(tokenizer, transcript, max_prompt_tokens)
        )
    }
    chat = [sys_msg, usr]
    prompt = TOKENIZER.apply_chat_template(chat, tokenize=False, add_generation_prompt=True)
    # Generate the synopsis using the engine
    text = ""
    async for chunk in engine.generate(prompt=prompt, sampling_params=SAMPLING_JSON):
        text = chunk.outputs[0].text
    data = json.loads(text.strip())

    def _as_list_str(v):
        if v is None:
            return []
        if isinstance(v, str):
            return [v] if v.strip() else []
        if isinstance(v, list):
            return [str(x) for x in v if str(x).strip()]
        return []

    # Validate the output against the schema
    data = {
        "focus": str(data.get("focus", "")),
        "user_goals": _as_list_str(data.get("user_goals")),
        "constraints": _as_list_str(data.get("constraints")),
        "key_entities": _as_list_str(data.get("key_entities")),
        "artifacts": _as_list_str(data.get("artifacts")),
        "open_questions": _as_list_str(data.get("open_questions")),
        "action_items": _as_list_str(data.get("action_items")),
        "decisions": _as_list_str(data.get("decisions")),
        "query_seed": str(data.get("query_seed", "")),
    }
    return MemorySynopsis(**data)


def _render_transcript(messages: List[Message], max_chars: int = 12000) -> str:
    """
    Linearized transcript with role tags. Keep it simple for robustness.
    """
    lines: List[str] = []
    for m in messages:
        role = (getattr(m, "role", None) or "user").lower()
        content = getattr(m, "content", "")
        # Strip any heavy reference appendix you add later
        content = content.split("<|begin_of_reference|>", 1)[0].strip()
        lines.append(f"{role.capitalize()}: {content}")
    text = "\n".join(lines)
    return text if len(text) <= max_chars else text[-max_chars:]


async def build_memory_synopsis(
        messages: List[Message],
        tokenizer: Any,
        engine: Any,
        prev_synopsis: Optional[MemorySynopsis] = None,
        max_prompt_tokens: int = 3500,
) -> MemorySynopsis:
    """
    Create/refresh the rolling MemorySynopsis.
    - messages: full chat so far (system/assistant/user)
    - prev_synopsis: prior memory to carry forward (we’ll merge)
    - mode: "llm" | "extractive"
    """
    transcript = _render_transcript(messages)
    cur = await llm_synopsis_from_transcript(engine, tokenizer, transcript, max_prompt_tokens=max_prompt_tokens)
    if prev_synopsis:
        cur = _merge_synopses(prev_synopsis, cur)

    # tighten fields (keep stable, short)
    cur.focus = _truncate_sentence(cur.focus, 180)
    cur.query_seed = _truncate_sentence(cur.query_seed or cur.focus, 300)
    cur.user_goals = cur.user_goals[:8]
    cur.constraints = cur.constraints[:8]
    cur.key_entities = cur.key_entities[:16]
    cur.artifacts = cur.artifacts[:16]
    cur.open_questions = cur.open_questions[:8]
    cur.action_items = cur.action_items[:8]
    cur.decisions = cur.decisions[:8]
    return cur


def _truncate_sentence(s: str, max_chars: int) -> str:
    return s if len(s) <= max_chars else s[:max_chars - 1] + "…"


_STOPWORDS = set("""
a an the and or but if then else for while of in on to from by with without within into across around 
through over under is are was were be been being do does did done has have had having as at it its this 
that these those i you he she they we me him her them my your our their mine yours ours theirs not no yes 
can could should would may might will just very really about above below between after before during 
again further once here there when where why how all any both each few more most other some such nor 
only own same so than too very s t don should’ve now d ll m o re ve y ain aren couldn didn doesn hadn 
hasn haven isn ma mightn mustn needn shan shouldn wasn weren won wouldn
""".split())


def _tokens(s: str) -> List[str]:
    return [w for w in re.findall(r"\w+", s.lower()) if w not in _STOPWORDS]


def _overlap(a: str, b: str) -> float:
    a_set, b_set = set(_tokens(a)), set(_tokens(b))
    if not a_set or not b_set:
        return 0.0
    return len(a_set & b_set) / len(a_set | b_set)


def _novelty(new: str, old: str) -> float:
    """How much NEW content (unique tokens) the `new` string contributes vs old."""
    a_set, b_set = set(_tokens(new)), set(_tokens(old))
    if not a_set:
        return 0.0
    return len(a_set - b_set) / len(a_set)


def _informativeness(s: str) -> float:
    """Rough signal: longer strings, with digits/symbols, score higher."""
    s = s.strip()
    if not s:
        return 0.0
    score = len(s)
    if any(ch.isdigit() for ch in s):
        score += 10
    if any(ch in "/-_.:%" for ch in s):
        score += 5
    return score


def _canon_item(x: str) -> str:
    """Canonicalize for dedupe (casefold + trim + normalize URLs lightly)."""
    c = x.strip().casefold()
    c = re.sub(r"https?://(www\.)?", "", c)
    c = re.sub(r"/+$", "", c)
    return c


def _is_near_substring(a: str, b: str) -> bool:
    """Treat very similar items as duplicates if one is (almost) contained in the other."""
    a_c, b_c = _canon_item(a), _canon_item(b)
    if a_c == b_c:
        return True
    # soft containment with word boundaries
    return (a_c in b_c and len(a_c) >= 0.8 * len(b_c)) or (b_c in a_c and len(b_c) >= 0.8 * len(a_c))


def _stable_unique(items: Iterable[str], cap: int) -> List[str]:
    out, seen = [], []
    for x in items:
        x = x.strip()
        if not x:
            continue
        if any(_is_near_substring(x, y) for y in seen):
            continue
        seen.append(x)
        out.append(x)
        if len(out) >= cap:
            break
    return out


def _interleave(a: List[str], b: List[str]) -> Iterable[str]:
    """Yield new-first interleaving to reduce old bias."""
    for i in range(max(len(a), len(b))):
        if i < len(b):  # NEW first
            yield b[i]
        if i < len(a):
            yield a[i]


def _rank_items_new_first(old: List[str], new: List[str]) -> List[str]:
    """Rank by: (1) comes from NEW, (2) informativeness desc, (3) original order."""
    combined = [(x, 'old', i, _informativeness(x)) for i, x in enumerate(old)] + \
               [(x, 'new', i, _informativeness(x)) for i, x in enumerate(new)]
    # Stable sort: new > old, info desc, original index asc
    combined.sort(key=lambda t: (t[1] != 'new', -t[3], t[2]))
    return [t[0] for t in combined]


def _merge_list(old: List[str], new: List[str], cap: int) -> List[str]:
    # Prefer items from NEW overall, then informativeness, then order; dedupe softly.
    ranked = _rank_items_new_first(old, new)
    return _stable_unique(ranked, cap=cap)


def _pick_text(old: str, new: str) -> str:
    """
    Balanced chooser for focus/query_seed:
    - Prefer NEW if it adds meaningful novelty OR is much more informative.
    - Otherwise keep OLD (stability).
    """
    a, b = old.strip(), new.strip()
    if not a:
        return b
    if not b:
        return a

    ov = _overlap(a, b)
    nov = _novelty(b, a)
    info_a, info_b = _informativeness(a), _informativeness(b)

    # Prefer new if it brings ≥30% novel tokens OR improves informativeness by ≥20%
    if nov >= 0.30 or (info_b >= 1.2 * info_a):
        return b

    # If new is longer and sufficiently overlaps, accept (your original behavior)
    if len(b) > len(a) and ov >= 0.40:
        return b

    # If old is very generic (too short) and new is concrete, accept new
    if len(a) < 40 and info_b > info_a:
        return b

    return a


def _merge_synopses(old: "MemorySynopsis", new: "MemorySynopsis") -> "MemorySynopsis":
    """
    Recency-aware merge:
    - Text fields use novelty+informativeness to avoid over-favoring old.
    - Lists favor NEW items first, with robust dedupe & caps.
    """
    return MemorySynopsis(
        focus=_pick_text(old.focus, new.focus),
        user_goals=_merge_list(old.user_goals, new.user_goals, cap=12),
        constraints=_merge_list(old.constraints, new.constraints, cap=12),
        key_entities=_merge_list(old.key_entities, new.key_entities, cap=24),
        artifacts=_merge_list(old.artifacts, new.artifacts, cap=24),
        open_questions=_merge_list(old.open_questions, new.open_questions, cap=12),
        action_items=_merge_list(old.action_items, new.action_items, cap=12),
        decisions=_merge_list(old.decisions, new.decisions, cap=12),
        # For query_seed, bias to NEW if it's sufficiently different or richer in constraints.
        query_seed=_pick_text(old.query_seed, new.query_seed)[:300],
    )


def build_query_seed_from_synopsis(s: MemorySynopsis) -> str:
    """
    Make a terse seed to prepend to your reformulation, e.g.:
    'User intent: … Constraints: … Entities: …'
    """
    pieces = []
    if s.focus:
        pieces.append(f"Intent: {s.focus}")
    if s.constraints:
        pieces.append("Constraints: " + ", ".join(s.constraints[:5]))
    if s.key_entities:
        pieces.append("Entities: " + ", ".join(s.key_entities[:6]))
    if s.artifacts:
        pieces.append("Artifacts: " + ", ".join(s.artifacts[:4]))
    if s.open_questions:
        pieces.append("Open: " + "; ".join(s.open_questions[:2]))
    seed = " | ".join(pieces)
    return seed if seed else s.query_seed or ""


def prepend_synopsis_system_message(messages: List[Message], synopsis: MemorySynopsis) -> List[Message]:
    """
    Non-invasive way to bias both reformulation + generation:
    insert a tiny system message carrying the synopsis.
    """
    content = (
            "Conversation synopsis (for retrieval & reasoning):\n"
            + synopsis.to_json()
            + "\nUse this to resolve pronouns and keep queries on-topic."
    )
    sys_msg = Message(role="system", content=content)
    return [sys_msg] + list(messages)
