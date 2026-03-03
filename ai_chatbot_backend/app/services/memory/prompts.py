"""
Memory synopsis prompts for conversation compression and merging.

Plain-string prompts used by synopsis.py for LLM calls.
"""

SYNOPSIS_SYSTEM = (
    "You are a memory-synopsis compressor. "
    "Given a chat transcript, produce a STRUCTURED JSON with keys: "
    "focus (string), user_goals (list[str]), constraints (list[str]), key_entities (list[str]), "
    "artifacts (list[str]), open_questions (list[str]), action_items (list[str]), decisions (list[str]). "
    "\nHere's some description of each key: \n"
    "focus - what is the main topic / intent of the user? \n"
    "user_goals - user's explicit goals/preferences. \n"
    "constraints - hard constraints (versions, dates, scope, etc.). \n"
    "key_entities - people, products, datasets, repos, courses... \n"
    "artifacts - files, URLs, IDs, paths mentioned. \n"
    "open_questions - unresolved questions the user asked. \n"
    "action_items - TODOs, 'next steps'. \n"
    "decisions - agreed choices so far. \n"
    "\nRules:\n"
    "- Return ONLY a single JSON object that matches the schema keys and types above.\n"
    "- Keep text terse and factual. No markdown, no code fences, no extra commentary.\n"
    "- Arrays must contain strings only; deduplicate items; remove empty strings.\n"
    "- Extract explicit constraints (versions, dates, scope limits) as strings.\n"
)

SYNOPSIS_USER_TEMPLATE = """Transcript:
{transcript}

Requirements:
- Summarize tersely.
- Deduplicate entities and URLs/paths.
- Extract explicit constraints (versions, dates, scope limits).
Return ONLY JSON.
"""

MERGE_SYSTEM = (
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

MERGE_USER_TEMPLATE = """OLD_SYNOPSIS:
{old_json}

NEW_SYNOPSIS:
{new_json}

Task:
Produce the single best merged synopsis following the rules. Return ONLY JSON.
"""
