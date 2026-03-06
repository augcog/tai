# JSON Schema definitions for structured output formats.
# Used with response_format parameter (OpenAI) or GuidedDecodingParams (VLLM)
# to guarantee structurally valid JSON output.

# ========================
# Memory Synopsis JSON Schema
# ========================
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

# ========================
# Response Blocks JSON Schema (for structured output mode)
# ========================

CITATION_SCHEMA = {
    "type": "object",
    "properties": {
        "id": {
            "type": "integer",
            "description": "Reference number from the provided context"
        },
        "quote_text": {
            "type": "string",
            "description": "Exact quoted text from the reference"
        },
    },
    "required": ["id", "quote_text"],
    "additionalProperties": False
}

BLOCK_SCHEMA = {
    "type": "object",
    "properties": {
        "citations": {
            "type": "array",
            "items": CITATION_SCHEMA,
            "description": "Citations referencing the provided context (1–2 per block). Split into multiple blocks when there are more quotes, even from the same source."
        },
        "open": {
            "type": "boolean",
            "description": "true to open the cited reference file on the learner's screen. false to keep current state."
        },
        "markdown_content": {
            "type": "string",
            "description": "Rich text content in Markdown format based on the citations above. For headings, include markdown hashes (e.g., '## Title') directly in markdown_content. For code blocks, include fenced Markdown with language identifier."
        },
        "close": {
            "type": "boolean",
            "description": "true to close the reference file after this block. false to keep it open for continued explanation."
        },
    },
    "required": ["citations", "open", "markdown_content", "close"],
    "additionalProperties": False
}

RESPONSE_BLOCKS_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "thinking": {
            "type": "string",
            "description": "Optional reasoning/scratchpad text. Leave empty when not needed.",
        },
        "blocks": {
            "type": "array",
            "items": BLOCK_SCHEMA,
            "description": "Array of content blocks forming the response"
        }
    },
    "required": ["thinking", "blocks"],
    "additionalProperties": False
}

# OpenAI response_format compatible schema
RESPONSE_BLOCKS_OPENAI_FORMAT = {
    "type": "json_schema",
    "json_schema": {
        "name": "chat_response_blocks",
        "strict": True,
        "schema": RESPONSE_BLOCKS_JSON_SCHEMA
    }
}

# ========================
# Voice Tutor JSON Schema (with unreadable property)
# ========================

VOICE_TUTOR_BLOCK_SCHEMA = {
    "type": "object",
    "properties": {
        "type": {
            "type": "string",
            "enum": ["readable", "not_readable"],
            "description": "\"readable\" for content that can be spoken aloud by TTS, \"not_readable\" for content that should only be shown visually (code, formulas, tables)."
        },
        "citations": {
            "type": "array",
            "items": CITATION_SCHEMA,
            "description": "Citations referencing the provided context."
        },
        "open": {
            "type": "boolean",
            "description": "true to open the cited reference file on the learner's screen. false to keep current state."
        },
        "markdown_content": {
            "type": "string",
            "description": "The content in Markdown format. For readable blocks, write text that can be read aloud naturally. For not_readable blocks, include code, formulas, or tables that are shown visually only."
        },
        "close": {
            "type": "boolean",
            "description": "true to close the reference file after this block. false to keep it open for continued explanation."
        },
    },
    "required": ["type", "citations", "open", "markdown_content", "close"],
    "additionalProperties": False
}

VOICE_TUTOR_RESPONSE_SCHEMA = {
    "type": "object",
    "properties": {
        "thinking": {
            "type": "string",
            "description": "Optional reasoning/scratchpad text. Leave empty when not needed.",
        },
        "blocks": {
            "type": "array",
            "items": VOICE_TUTOR_BLOCK_SCHEMA,
            "description": "Array of content blocks forming the response"
        }
    },
    "required": ["thinking", "blocks"],
    "additionalProperties": False
}

# OpenAI response_format compatible schema for voice tutor mode
VOICE_TUTOR_OPENAI_FORMAT = {
    "type": "json_schema",
    "json_schema": {
        "name": "voice_tutor_response_blocks",
        "strict": True,
        "schema": VOICE_TUTOR_RESPONSE_SCHEMA
    }
}

# ========================
# Outline JSON Schema (for tutor outline mode)
# ========================

OUTLINE_BULLET_SCHEMA = {
    "type": "object",
    "properties": {
        "point": {
            "type": "string",
            "description": "The teaching goal for this page — what the student should learn."
        },
        "purpose": {
            "type": "string",
            "description": (
                "A model-facing instruction for the downstream content generator. "
                "Describes HOW to explain this page's knowledge to the student: "
                "what pedagogical approach to use, what examples or analogies to include, "
                "what depth of detail to aim for, and how to structure the explanation. "
                "Not shown to students."
            )
        },
        "references": {
            "type": "array",
            "items": {"type": "integer"},
            "minItems": 1,
            "description": "Reference numbers that provide evidence for this teaching point. At least one required."
        },
    },
    "required": ["point", "purpose", "references"],
    "additionalProperties": False
}

OUTLINE_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "needs_multiple_pages": {
            "type": "boolean",
            "description": (
                "true if the student's question requires multiple pages to explain "
                "thoroughly (multi-step concepts, prerequisites, etc.). "
                "false if a single page is sufficient (simple factual question, definition, etc.). "
                "When false, still produce exactly one bullet."
            )
        },
        "title": {
            "type": "string",
            "description": "A clear, descriptive title for this teaching outline."
        },
        "outline": {
            "type": "array",
            "items": {"type": "string"},
            "description": (
                "High-level page titles only — one string per page. "
                "This is the clean outline the student sees before detailed content is generated. "
                "Each entry must match the corresponding bullet's 'point' field exactly."
            )
        },
        "bullets": {
            "type": "array",
            "items": OUTLINE_BULLET_SCHEMA,
            "description": "Ordered list of page teaching goals with supporting evidence."
        },
    },
    "required": ["needs_multiple_pages", "title", "outline", "bullets"],
    "additionalProperties": False
}

OUTLINE_OPENAI_FORMAT = {
    "type": "json_schema",
    "json_schema": {
        "name": "tutor_outline",
        "strict": True,
        "schema": OUTLINE_JSON_SCHEMA
    }
}

# ========================
# Page Bullets JSON Schema (for per-page sub-bullet generation via vLLM guided decoding)
# ========================

PAGE_SUB_BULLET_SCHEMA = {
    "type": "object",
    "properties": {
        "point": {
            "type": "string",
            "description": "A specific sub-topic or knowledge point to cover on this page."
        },
        "reference_ids": {
            "type": "array",
            "items": {"type": "integer"},
            "description": "Reference numbers from the provided materials that support this sub-point."
        },
    },
    "required": ["point", "reference_ids"],
    "additionalProperties": False
}

PAGE_BULLETS_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "sub_bullets": {
            "type": "array",
            "items": PAGE_SUB_BULLET_SCHEMA,
            "description": (
                "Ordered list of sub-topics for this page. Each sub-bullet is a specific "
                "knowledge point that the narration model will expand into explanatory text."
            )
        },
    },
    "required": ["sub_bullets"],
    "additionalProperties": False
}

# ========================
# Page Content JSON Schema (for OpenAI block-based page narration, TTS-aware)
# ========================

PAGE_CONTENT_TTS_BLOCK_SCHEMA = {
    "type": "object",
    "properties": {
        "type": {
            "type": "string",
            "enum": ["readable", "not_readable"],
            "description": (
                '"readable" for narration text that will be spoken aloud by TTS. '
                '"not_readable" for visual-only content (code, formulas, tables) '
                'shown on screen but not spoken.'
            )
        },
        "citations": {
            "type": "array",
            "items": CITATION_SCHEMA,
            "description": "Citations referencing the provided context (1–2 per block)."
        },
        "open": {
            "type": "boolean",
            "description": "true to open the cited reference file on the learner's screen. false to keep current state."
        },
        "markdown_content": {
            "type": "string",
            "description": (
                "For readable blocks: natural language narration written to be spoken aloud. "
                "For not_readable blocks: code, formulas, or tables displayed visually only."
            )
        },
        "close": {
            "type": "boolean",
            "description": "true to close the reference file after this block. false to keep it open."
        },
    },
    "required": ["type", "citations", "open", "markdown_content", "close"],
    "additionalProperties": False
}

PAGE_CONTENT_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "sub_bullets": {
            "type": "array",
            "items": {"type": "string"},
            "description": (
                "Knowledge sub-points for this page. If the page's teaching goal is "
                "straightforward, this can be a single entry matching the page title. "
                "If the topic needs decomposition, list 2–5 specific sub-points. "
                "These are shown to the student as a mini table-of-contents before the narration."
            )
        },
        "blocks": {
            "type": "array",
            "items": PAGE_CONTENT_TTS_BLOCK_SCHEMA,
            "description": (
                "Ordered narration blocks. Each block declares its type (readable or not_readable), "
                "can open/close a reference file, and contains markdown content. Readable blocks "
                "are spoken aloud by TTS; not_readable blocks are displayed visually only."
            )
        },
    },
    "required": ["sub_bullets", "blocks"],
    "additionalProperties": False
}

PAGE_CONTENT_OPENAI_FORMAT = {
    "type": "json_schema",
    "json_schema": {
        "name": "page_content_blocks",
        "strict": True,
        "schema": PAGE_CONTENT_JSON_SCHEMA
    }
}
