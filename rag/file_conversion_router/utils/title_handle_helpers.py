"""
Helper functions and schema builders for title_handle.py

This module extracts reusable components to improve readability while preserving
all original prompts and schemas.
"""

import re
from typing import Dict, List, Any


# =============================================================================
# CONFIGURATION CONSTANTS
# =============================================================================

MAX_KEY_CONCEPTS = 5
MAX_RECAP_QUESTIONS = 5
MAX_SECTIONS = 5
QUESTION_OPTIONS_COUNT = 4


# =============================================================================
# SCHEMA BUILDERS - Reusable Schema Components
# =============================================================================

def build_check_in_question_schema() -> Dict[str, Any]:
    """
    Build the check-in question schema used across multiple functions.

    Returns:
        Complete schema dict for check-in questions with all field descriptions
    """
    return {
        "type": "object",
        "properties": {
            "question_text": {
                "type": "string",
                "description": "The full text of the multiple-choice question."
            },
            "options": {
                "type": "array",
                "items": {"type": "string"},
                "minItems": QUESTION_OPTIONS_COUNT,
                "description": f"An array containing exactly {QUESTION_OPTIONS_COUNT} possible answers."
            },
            "correct_answer": {
                "type": "array",
                "items": {"type": "integer"},
                "description": "The indices of the correct options in the options array, e.g. [0,1] for the first and second options."
            },
            "explanation": {
                "type": "string",
                "description": "A detailed explanation describing why the correct answer is right and why the other options are incorrect."
            }
        },
        "required": ["question_text", "options", "correct_answer", "explanation"],
        "additionalProperties": False
    }


def build_content_coverage_schema() -> Dict[str, Any]:
    """
    Build the content coverage schema.

    Returns:
        Schema for content coverage array
    """
    return {
        "type": "array",
        "description": "List only the aspects that the section actually explained with aspect and content.",
        "items": {
            "type": "object",
            "properties": {
                "aspect": {
                    "type": "string",
                    "minLength": 1
                },
                "content": {"type": "string"}
            },
            "required": ["aspect", "content"],
            "additionalProperties": False
        }
    }


def build_recap_questions_schema() -> Dict[str, Any]:
    """
    Build the recap questions schema with all validation rules.

    Returns:
        Complete schema for recap questions array
    """
    return {
        "type": "array",
        "items": {
            "type": "object",
            "properties": {
                "question_id": {
                    "type": "integer",
                    "description": "Sequential number starting from 1"
                },
                "question_type": {
                    "type": "string",
                    "enum": ["recap"]
                },
                "question_text": {
                    "type": "string",
                    "description": "The complete text of the multiple-choice recap question"
                },
                "options": {
                    "type": "array",
                    "items": {"type": "string"},
                    "minItems": QUESTION_OPTIONS_COUNT,
                    "maxItems": QUESTION_OPTIONS_COUNT,
                    "description": f"Exactly {QUESTION_OPTIONS_COUNT} possible answers"
                },
                "correct_answers": {
                    "type": "array",
                    "items": {"type": "integer"},
                    "minItems": 1,
                    "maxItems": 2,
                    "description": "Indices of correct options (0-3), minimum 1, maximum 2"
                },
                "explanation": {
                    "type": "string",
                    "description": "Detailed explanation of why the correct answers are right and others are wrong"
                },
                "knowledge_areas": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Key knowledge areas this question tests"
                }
            },
            "required": [
                "question_id", "question_type", "question_text",
                "options", "correct_answers", "explanation", "knowledge_areas"
            ],
            "additionalProperties": False
        },
        "minItems": 0,
        "maxItems": MAX_RECAP_QUESTIONS,
        "description": f"0-{MAX_RECAP_QUESTIONS} recap questions for review and self-check, ordered from easy to moderate"
    }


def build_sub_problem_schema(description: str, example_question: str) -> Dict[str, Any]:
    """
    Build a sub-problem schema with custom description.

    Args:
        description: Description of the sub-problem's purpose
        example_question: Example question to include in description

    Returns:
        Complete sub-problem schema
    """
    return {
        "type": "object",
        "properties": {
            "description_of_problem": {
                "type": "string",
                "description": f'{description} e.g. "{example_question}"'
            },
            "options": {
                "type": "array",
                "items": {"type": "string"},
                "minItems": 2,
                "description": "The options for the sub problem, they should be key_concepts in the md_content"
            },
            "answers_options": {
                "type": "array",
                "items": {"type": "integer"},
                "description": "The index of the option in the options array, e.g. [0,1] for the first and second options"
            },
            "explanation_of_answer": {
                "type": "string",
                "description": "The explanation of why this option is the answer, it should be a key_concept in the md_content"
            }
        },
        "required": ["description_of_problem", "options", "answers_options", "explanation_of_answer"],
        "additionalProperties": False
    }


def build_problems_schema() -> Dict[str, Any]:
    """
    Build the complete problems schema with both sub-problems.

    Returns:
        Complete problems array schema
    """
    return {
        "type": "array",
        "items": {
            "type": "object",
            "properties": {
                "ID": {
                    "type": "string",
                    "description": 'Which Exercise id this problem belongs to, e.g. "Exercise 1","Challenge 1"'
                },
                "content": {
                    "type": "string",
                    "description": "The whole content of Exercise or Challenge. Do not miss or change any part of the description."
                },
                "sub_problem_1": build_sub_problem_schema(
                    "A multiple-choice question that better contains at least two correct options. Its purpose is to help students understand the problem statement and identify the underlying concepts.",
                    "What knowledge are involved in this problem?"
                ),
                "sub_problem_2": build_sub_problem_schema(
                    "A multiple-choice question that better contain at least two correct options. Its purpose is to guide students in designing an approach or solution framework.",
                    "Which methods could be applied to solve this problem effectively?"
                )
            },
            "required": ["ID", "content", "sub_problem_1", "sub_problem_2"],
            "additionalProperties": False
        }
    }


def build_speaker_schema() -> Dict[str, Any]:
    """
    Build the speaker identification schema with role rules.

    Returns:
        Complete speaker array schema with numbering rules
    """
    return {
        "type": "array",
        "description": "A list of speakers identified in the content along with their inferred roles.",
        "items": {
            "type": "object",
            "properties": {
                "speaker_id": {
                    "type": "string",
                    "description": "The unique identifier for the speaker, e.g., 'Speaker_00'."
                },
                "role": {
                    "type": "string",
                    "description": "The speaker identifier. PRIORITY: Use actual name if speaker introduces themselves (e.g., 'I am John', 'My name is Sarah'). FALLBACK: Use numbered roles (Professor, TA_1, TA_2, Student_1, Student_2, Unknown_1, etc.) only if no name is detected."
                }
            },
            "required": ["speaker_id", "role"],
            "additionalProperties": False
        }
    }


def build_key_concepts_schema(title_list: List[str]) -> Dict[str, Any]:
    """
    Build the key concepts schema with title enumeration.

    Args:
        title_list: List of valid section titles

    Returns:
        Complete key concepts array schema
    """
    return {
        "type": "array",
        "items": {
            "type": "object",
            "properties": {
                "concepts": {"type": "string"},
                "source_section_title": {
                    "type": "string",
                    "enum": title_list,
                    "description": "The single, exact title from title_list that this concept is derived from."
                },
                "check_in_question": build_check_in_question_schema(),
                "content_coverage": build_content_coverage_schema()
            },
            "required": [
                "concepts",
                "source_section_title",
                "check_in_question",
                "content_coverage"
            ],
            "additionalProperties": False
        }
    }


def build_paragraphs_schema(paragraph_count: int) -> Dict[str, Any]:
    """
    Build the paragraphs schema for structuring content.

    Args:
        paragraph_count: Number of paragraphs in the content

    Returns:
        Complete paragraphs array schema
    """
    return {
        "type": "array",
        "minItems": paragraph_count,
        "maxItems": paragraph_count,
        "items": {
            "type": "object",
            "properties": {
                "title": {"type": "string"},
                "paragraph_index": {
                    "type": "integer",
                    "description": "The 1-based index from the paragraphs array where this paragraph is located.",
                    "maximum": paragraph_count
                }
            },
            "required": ["title", "paragraph_index"],
            "additionalProperties": False
        }
    }


def build_sections_schema(paragraph_count: int) -> Dict[str, Any]:
    """
    Build the sections schema for grouping paragraphs.

    Args:
        paragraph_count: Total number of paragraphs (for validation)

    Returns:
        Complete sections array schema
    """
    return {
        "type": "array",
        "maxItems": MAX_SECTIONS,
        "items": {
            "type": "object",
            "properties": {
                "section_title": {"type": "string"},
                "start_paragraph_index": {
                    "type": "integer",
                    "description": "The 1-based index from the paragraphs array where this section begins.",
                    "minimum": 1,
                    "maximum": paragraph_count
                }
            },
            "required": ["section_title", "start_paragraph_index"],
            "additionalProperties": False
        }
    }


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def prepare_title_list(index_helper: List[Dict[str, Any]]) -> List[str]:
    """
    Prepare title list from index helper, replacing quotes.

    Args:
        index_helper: List of dictionaries containing title information

    Returns:
        List of title strings with normalized quotes
    """
    return [key.replace('"', "'") for d in index_helper for key in d.keys()]


def count_paragraphs(md_text: str) -> int:
    """
    Count the number of paragraphs in markdown text.

    Args:
        md_text: The markdown text to analyze

    Returns:
        Number of paragraphs found
    """
    md_text = md_text.strip()
    blocks = re.split(r'\n\s*\n', md_text)
    paragraphs = [b for b in blocks if b.strip()]
    return len(paragraphs)
