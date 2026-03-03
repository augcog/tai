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
        "title": {
            "type": "string",
            "description": "A clear, descriptive title for this teaching outline."
        },
        "bullets": {
            "type": "array",
            "items": OUTLINE_BULLET_SCHEMA,
            "description": "Ordered list of page teaching goals with supporting evidence."
        },
    },
    "required": ["title", "bullets"],
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
