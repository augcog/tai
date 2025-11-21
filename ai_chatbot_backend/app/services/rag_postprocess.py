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


LTM_SCHEMA = {
    "type": "object",
    "properties": {
        "user_id": {"type": "string"},
        "knowledge_profile": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "course_id": {"type": "string"},
                    "weak_topics": {"type": "array", "items": {"type": "string"}}
                },
                "required": ["course_id", "weak_topics"],
                "additionalProperties": False
            }
        },        
        "current_focus": {"type": "array", "items": {"type": "string"}},
        "learning_preferences": {"type": "array", "items": {"type": "string"}},
        "user_profile": {"type": "array", "items": {"type": "string"}},
    },
    "required": ["user_id", "knowledge_profile", "current_focus", "learning_preferences", "user_profile"],
    "additionalProperties": False
}
GUIDED_LTM = GuidedDecodingParams(json=LTM_SCHEMA)
SAMPLING_JSON_LTM = SamplingParams(
    temperature=0.0, top_p=1.0, max_tokens=8000, guided_decoding=GUIDED_LTM, skip_special_tokens=False
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


from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any
import json

# Helper structure for the 'knowledge_profile' array items
@dataclass
class KnowledgeProfileItem:
    """Represents a specific course and the user's weak topics within it."""
    course_id: str = ""
    weak_topics: List[str] = field(default_factory=list)

@dataclass
class MemorySynopsisLong:
    """
    Structured memory containing the user's long-term learning profile (LTM_SCHEMA).
    """
    user_id: str = ""
    # knowledge_profile is a list of KnowledgeProfileItem objects
    knowledge_profile: List[KnowledgeProfileItem] = field(default_factory=list)
    current_focus: List[str] = field(default_factory=list)
    learning_preferences: List[str] = field(default_factory=list)
    user_profile: List[str] = field(default_factory=list)

    def to_json(self) -> str:
        """Converts the dataclass instance to a JSON string."""
        # Custom serialization logic to handle the nested dataclass (KnowledgeProfileItem)
        data = asdict(self)
        return json.dumps(data, ensure_ascii=False)

    @staticmethod
    def from_json(s: str) -> "MemorySynopsisLong":
        """Creates a MemorySynopsisLong instance from a JSON string."""
        data: Dict[str, Any] = json.loads(s or "{}")
        
        # Manually deserialize the nested 'knowledge_profile' list of objects
        knowledge_profiles_data = data.get("knowledge_profile", [])
        knowledge_profiles = [
            KnowledgeProfileItem(**item)
            for item in knowledge_profiles_data
        ]
        
        # Remove the raw list data and insert the deserialized objects
        data["knowledge_profile"] = knowledge_profiles
        
        # Use a safe method to initialize, filling defaults if keys are missing
        return MemorySynopsisLong(**{k: data.get(k, v) for k, v in asdict(MemorySynopsisLong()).items()})


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
        print("[INFO] Merging with previous MemorySynopsis...")
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


async def _llm_synthesize_ltm(
        engine: Any,
        tokenizer: Any,
        new_stm: MemorySynopsis,
        prev_ltm: Optional[MemorySynopsisLong],
        max_prompt_tokens: int = 3500,
) -> MemorySynopsisLong:
    """
    使用 LLM 根据聊天记录、STM 和可选的现有 LTM 生成新的 LTM JSON。
    """
    # 准备 LTM、STM 和转录文本
    existing_ltm_json = prev_ltm.to_json() if prev_ltm else "{}"
    print(f"Existing LTM JSON: {existing_ltm_json}")
    new_stm_json = new_stm.to_json()

    # LTM_SYSTEM 中提到的输入数据结构
    LLM_USER_LTM_TEMPLATE = """
EXISTING_LTM:
{existing_ltm_json}

NEW_STM:
{new_stm_json}

Task:
Produce the single best updated LTM JSON object following the system rules. Return ONLY JSON. Please do not leave your output an empty json skeleton with no content.
"""

    # 准备系统和用户消息
    sys_msg = {"role": "system", "content": _LLM_SYSTEM_LTM}
    print(f"STM : {new_stm_json}")
    usr_content = LLM_USER_LTM_TEMPLATE.format(
        existing_ltm_json=existing_ltm_json,
        new_stm_json=new_stm_json,
    )
    print(f"usr_content for LTM synthesis: {usr_content}")
    usr = {"role": "user",
           "content": usr_content}
    # usr = {
    #     "role": "user",
    #     "content": _LLM_USER_TEMPLATE.format(
    #         transcript=_truncate_to_tokens(tokenizer, transcript, max_prompt_tokens)
    #     )
    # }
    chat = [sys_msg, usr]
    # sys_msg = {"role": "system", "content": _LLM_MERGE_SYSTEM}
    # usr_msg = {"role": "user", "content": _LLM_MERGE_USER_TEMPLATE.format(old_json=old_json, new_json=new_json)}
    prompt = tokenizer.apply_chat_template(chat, tokenize=False, add_generation_prompt=True)

    # 使用 LTM 专用的采样参数（特别是 GUIDED_LTM）进行生成
    text = ""
    print(f"sampling params for LTM: {SAMPLING_JSON_LTM}")
    print(f"LTM synthesis prompt: {prompt}")
    print(f"\n\n\nstart generating LTM \n\n\n")
    async for chunk in engine.generate(
            prompt=prompt,
            sampling_params=SAMPLING_JSON_LTM,
            request_id=str(time.time_ns()) # 假设 time.time_ns() 可用
    ):
        text = chunk.outputs[0].text

    # 提取 JSON 结果
    # text = extract_channels(text).get('final', "{}")
    print('Generated MemorySynopsisLong JSON:', text)

    try:
        # 使用 MemorySynopsisLong.from_json 方法创建实例
        return MemorySynopsisLong.from_json(text.strip())
    except Exception as e:
        print(f'Failed to parse generated MemorySynopsisLong JSON: {e} | Text: {text}')
        # 如果解析失败，返回一个空的 LTM 实例
        return MemorySynopsisLong()

# --- 主要函数：构建 LTM ---

async def build_memory_synopsis_long(
        messages: List[Message],
        tokenizer: Any,
        engine: Any,
        new_stm: MemorySynopsis, # 新增：必须是已生成的 Short-Term Memory
        prev_synopsis_long: Optional[MemorySynopsisLong] = None, # LTM 历史
        chat_history_sid: Optional[str] = None,
        max_prompt_tokens: int = 3500,
) -> MemorySynopsisLong:
    """
    根据聊天记录、新生成的 STM 和可选的现有 LTM 生成/更新 Long-Term Memory (LTM)。
    - messages: 完整的聊天记录 (用于上下文推理)
    - new_stm: 最近会话生成的 MemorySynopsis (Short-Term Memory)
    - prev_synopsis_long: 之前的 LTM 实例 (MemorySynopsisLong)
    - chat_history_sid: 可选，如果提供了，尝试从数据库获取 prev_synopsis_long
    """

    # 1. 优雅地从数据库检索之前的 LTM (如果需要)
    # 此处省略 MongoDB 服务的具体导入和实现，仅保留逻辑结构。
    if chat_history_sid and not prev_synopsis_long:
        print("[INFO] Attempting to retrieve previous memory from DB...")
        try:
            from app.services.memory_synopsis_service import MemorySynopsisServiceLong
            memory_service = MemorySynopsisServiceLong()
            prev_synopsis_long = await memory_service.get_by_user_id(chat_history_sid)
        except Exception as e:
            print(f"[INFO] Failed to retrieve previous memory, generating from scratch: {e}")
            prev_synopsis_long = None  # Continue without previous memory

    # 2. 渲染转录文本 (Transcript)
    transcript = _render_transcript(messages)
    print(f"[INFO] previous LTM: {prev_synopsis_long}")
    # 3. 调用 LLM 合成新的 LTM
    updated_ltm = await _llm_synthesize_ltm(
        engine,
        tokenizer,
        new_stm,
        prev_synopsis_long,
        max_prompt_tokens=max_prompt_tokens
    )

    # 4. （可选）对 LTM 字段进行后处理/规范化 (类似于 STM 的 tighten fields)
    # LTM 结构复杂，通常由 LLM 严格按照 JSON Schema 生成，
    # 但可以添加额外的清理或大小限制，例如：
    # updated_ltm.historical_interactions = updated_ltm.historical_interactions[-20:] # 只保留最近20条会话记录

    return updated_ltm


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

_LLM_SYSTEM_LTM = (
    "You are a Long-Term Memory (LTM) Synthesis Agent for a teaching platform."
    "Given the student's existing long-term memory, the new short-term memory from the latest session, and the full chat transcript, your task is to produce an updated LTM JSON that reflects the student's profile, knowledge state, and learning history."

    "Input Data:"
    "1. EXISTING_LTM: The student's current comprehensive LTM JSON. (May be empty/null if this is the first session)."
    "2. NEW_STM: The structured Short-Term Memory JSON from the recent session."
    "3. CHAT_HISTORY: The full text transcript of the conversation that generated the NEW_STM."

    "LTM JSON Keys and Update Logic:"

    "1. knowledge_profile:"
    "Mastery/Confidence - Analyze the `CHAT_HISTORY`, `NEW_STM`'s `focus`, and `decisions`. If the student confidently explained a topic, successfully answered a question, or the session concluded with a clear understanding, update or increase the `confidence_score` for the related **concept** in `concept_mastery_level`. If a concept is newly mastered, add it to `mastered_topics`. Update `last_reviewed` for all affected concepts."
    "Gaps/Weakness - Analyze `open_questions`, any apparent confusion in the `CHAT_HISTORY`, or recurring struggles in the `NEW_STM`'s `focus`. Add or reinforce these areas in `weak_topics` and/or lower the `confidence_score` in `concept_mastery_level`."

    "2. learning_preferences:"
    "Style/Pace - Infer the student's preferred_learning_style (e.g., analogy-based, theoretical depth, or practical examples) and pace (fast, slow, detailed) from the tone and type of questions/requests in the `CHAT_HISTORY`."
    "Interests/Files - Consolidate `user_goals` and `key_entities` for preferred_topics_and_interests. Extract any mentioned file extensions, formats, or resource types from `artifacts` to populate **preferred_file_types."

    "3. historical_interactions:"
    "Add a new entry representing the current session. Use the `focus`, `artifacts`, and `decisions` from the `NEW_STM`. Assign a unique `session_id` and set `session_type` to 'chat'."

    "4. current_focus_and_goals:"
    "Consolidation - Merge the `user_goals` from `NEW_STM` into long_term_user_goals, removing goals that were completed (indicated by `decisions` or absence in future STMs)."
    "Recurrence - Aggregate `key_entities` into key_entities_of_interest. Track recurring unresolved questions from `open_questions` into persistent_open_questions."

    "Rules:"
    "- Return ONLY a single updated LTM JSON object that strictly matches the provided schema and type requirements."
    "- Aggregation: When updating arrays (e.g., `mastered_topics`, `long_term_user_goals`), deduplicate items and merge with the `EXISTING_LTM` content. Do not lose historical information unless it's explicitly resolved."
    "- Terseness: Keep all text entries (e.g., concept names, goals) terse and factual. No markdown, code fences, or extra commentary."
    "- Ensure all required keys are present and follow their specified formats (e.g., `confidence_score` as a number 0.0-1.0)."
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
    # print(f"MemorySynopsis prompt tokens: {prompt}")
    # Generate the synopsis using the engine
    text = ""
    async for chunk in engine.generate(
            prompt=prompt,
            sampling_params=SAMPLING_JSON,
            request_id=str(time.time_ns())
    ):
        text = chunk.outputs[0].text
    # text = extract_channels(text).get('final', "{}")
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
    print(f"Old MemorySynopsis JSON: {old_json}")
    print(f"New MemorySynopsis JSON: {new_json}")
    print(f"sys_msg for merge STM: {sys_msg}")
    print(f"usr_msg for merge STM: {usr_msg}")
    prompt = tokenizer.apply_chat_template([sys_msg, usr_msg], tokenize=False, add_generation_prompt=True)
    # Generate the merged synopsis using the engine
    text = ""
    # print(f"Merge MemorySynopsis prompt STM: {prompt}")
    # print(f"sampling params for merge STM: {SAMPLING_JSON}")
    async for chunk in engine.generate(
            prompt=prompt,
            sampling_params=SAMPLING_JSON,
            request_id=str(time.time_ns())
    ):
        text = chunk.outputs[0].text
    # text = extract_channels(text).get('final', "{}")
    # try to parse JSON, if fails return empty MemorySynopsis
    try:
        data = json.loads(text.strip())
    except json.JSONDecodeError:
        print('Failed to parse merged MemorySynopsis JSON:', text)
        return MemorySynopsis()
    return MemorySynopsis(**data)

