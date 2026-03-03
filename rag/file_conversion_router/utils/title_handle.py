"""
Title and content handling module for educational material processing.

This module provides functionality to process markdown content from educational materials,
extract key concepts, generate assessment questions, and manage speaker roles in transcripts.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path
from textwrap import dedent
from enum import Enum
import os
import re
import json
import yaml

from dotenv import load_dotenv
from openai import OpenAI
from loguru import logger


# ========================
# Configuration & Constants
# ========================

# Standard .env file location for RAG component
RAG_ENV_PATH = Path(__file__).parent.parent.parent / ".env"


def get_openai_api_key() -> Optional[str]:
    """
    Get OpenAI API key from the RAG component's .env file.

    Returns:
        The API key string if found, None otherwise.
    """
    # Try loading from the standard RAG .env location
    if RAG_ENV_PATH.exists():
        load_dotenv(dotenv_path=RAG_ENV_PATH, override=True)
    else:
        # Fallback to default load_dotenv behavior
        load_dotenv(override=True)

    api_key = os.getenv("OPENAI_API_KEY")

    if api_key and api_key.strip():
        return api_key.strip()

    return None

class SpeakerRole(Enum):
    """Enum for standardized speaker roles."""
    PROFESSOR = "Professor"
    TEACHING_ASSISTANT_PREFIX = "TA_"
    STUDENT_PREFIX = "Student_"
    UNKNOWN_PREFIX = "Unknown_"


@dataclass
class ProcessingConfig:
    """Configuration for content processing."""
    model: str = "gpt-4.1"
    temperature: float = 0.1
    max_key_concepts: int = 5
    max_recap_questions: int = 5
    max_time_gap_seconds: float = 5.0
    max_words_per_group: int = 200
    min_paragraphs_for_sections: int = 5
    max_sections: int = 5


@dataclass
class QuestionConfig:
    """Configuration for question generation."""
    options_count: int = 4
    min_correct_answers: int = 1
    max_correct_answers: int = 2


# ========================
# OpenAI Client Wrapper
# ========================

class OpenAIClientWrapper:
    """Wrapper for OpenAI API interactions."""

    def __init__(self, api_key: Optional[str] = None, lazy_init: bool = False):
        """Initialize OpenAI client with API key.

        Args:
            api_key: Optional API key to use
            lazy_init: If True, delay initialization until first use
        """
        self._api_key = api_key
        self._client = None
        self._config = None
        self._lazy_init = lazy_init

        if not lazy_init:
            self._initialize()

    def _initialize(self):
        """Initialize the OpenAI client."""
        if self._client is not None:
            return

        api_key = self._api_key
        if api_key is None:
            api_key = get_openai_api_key()

        if not api_key:
            raise ValueError("OpenAI API key not found in environment variables")

        self._client = OpenAI(api_key=api_key)
        self._config = ProcessingConfig()

    @property
    def client(self):
        """Get the OpenAI client, initializing if necessary."""
        if self._client is None:
            self._initialize()
        return self._client

    @property
    def config(self):
        """Get the processing config, initializing if necessary."""
        if self._config is None:
            self._initialize()
        return self._config

    def create_completion(
        self,
        system_prompt: str,
        user_content: str,
        response_format: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Create a completion with structured output.

        Args:
            system_prompt: System message for the completion
            user_content: User content to process
            response_format: JSON schema for response format

        Returns:
            Parsed JSON response from the API
        """
        try:
            response = self.client.chat.completions.create(
                model=self.config.model,
                temperature=self.config.temperature,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_content}
                ],
                response_format=response_format
            )

            content = response.choices[0].message.content
            return json.loads(content)

        except Exception as e:
            logger.error(f"OpenAI API error: {str(e)}")
            raise


# ========================
# JSON Schema Factory
# ========================

class SchemaFactory:
    """Factory for creating JSON schemas for different content types."""

    def __init__(self, config: Optional[ProcessingConfig] = None):
        """Initialize schema factory with configuration."""
        self.config = config or ProcessingConfig()
        self.question_config = QuestionConfig()

    def _create_check_in_question_schema(self) -> Dict[str, Any]:
        """Create schema for check-in questions."""
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
                    "minItems": self.question_config.options_count,
                    "description": f"An array containing exactly {self.question_config.options_count} possible answers."
                },
                "correct_answer": {
                    "type": "array",
                    "items": {"type": "integer"},
                    "description": "The indices of the correct options in the options array"
                },
                "explanation": {
                    "type": "string",
                    "description": "A detailed explanation of why answers are correct/incorrect"
                }
            },
            "required": ["question_text", "options", "correct_answer", "explanation"],
            "additionalProperties": False
        }

    def _create_aspects_schema(self) -> Dict[str, Any]:
        """Create schema for key concept aspects - multiple analytical perspectives."""
        return {
            "type": "array",
            "description": "Multiple detailed analytical descriptions of the key concept from different perspectives",
            "items": {
                "type": "object",
                "properties": {
                    "aspect": {
                        "type": "string",
                        "minLength": 1,
                        "description": "The type of analysis (e.g., definition, example, use case, implication)"
                    },
                    "content": {
                        "type": "string",
                        "description": "Detailed description analyzing the key concept from this perspective"
                    }
                },
                "required": ["aspect", "content"],
                "additionalProperties": False
            }
        }

    def _create_recap_question_schema(self) -> Dict[str, Any]:
        """Create schema for recap questions."""
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
                        "minItems": self.question_config.options_count,
                        "maxItems": self.question_config.options_count,
                        "description": f"Exactly {self.question_config.options_count} possible answers"
                    },
                    "correct_answers": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "minItems": self.question_config.min_correct_answers,
                        "maxItems": self.question_config.max_correct_answers,
                        "description": "Indices of correct options (0-3)"
                    },
                    "explanation": {
                        "type": "string",
                        "description": "Detailed explanation of answers"
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
            "maxItems": self.config.max_recap_questions,
            "description": "0-5 recap questions for review and self-check"
        }

    def _create_speaker_schema(self) -> Dict[str, Any]:
        """Create schema for speaker identification."""
        return {
            "type": "array",
            "description": "A list of speakers identified in the content",
            "items": {
                "type": "object",
                "properties": {
                    "speaker_id": {
                        "type": "string",
                        "description": "The unique identifier for the speaker"
                    },
                    "role": {
                        "type": "string",
                        "description": "The speaker role or actual name if detected"
                    }
                },
                "required": ["speaker_id", "role"],
                "additionalProperties": False
            }
        }

    def create_ipynb_schema(self, title_list: List[str]) -> Dict[str, Any]:
        """Create schema for ipynb content processing."""
        return {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "file_description": {
                            "type": "string",
                            "description": "A short summary describing the file type and key content (e.g., 'A lecture recording for recursion, it mentioned base cases and tree structures')"
                        },
                        "key_concepts": self._create_key_concepts_schema(title_list),
                        "problems": self._create_problems_schema(),
                        "recap_questions": self._create_recap_question_schema()
                    },
                    "required": ["file_description", "key_concepts", "problems", "recap_questions"],
                    "additionalProperties": False
                }
            }
        }

    def _create_key_concepts_schema(self, title_list: List[str]) -> Dict[str, Any]:
        """Create schema for key concepts."""
        return {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "concept": {
                        "type": "string",
                        "description": "Short keyword phrase or level-1 section title"
                    },
                    "source_section_title": {
                        "type": "string",
                        "enum": title_list,
                        "description": "The exact title from title_list where this concept was mentioned"
                    },
                    "aspects": self._create_aspects_schema(),
                    "check_in_question": self._create_check_in_question_schema()
                },
                "required": [
                    "concept", "source_section_title",
                    "aspects", "check_in_question"
                ],
                "additionalProperties": False
            }
        }

    def _create_problems_schema(self) -> Dict[str, Any]:
        """Create schema for problems section."""
        return {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "ID": {
                        "type": "string",
                        "description": 'Exercise ID (e.g., "Exercise 1", "Challenge 1")'
                    },
                    "content": {
                        "type": "string",
                        "description": "The whole content of Exercise or Challenge"
                    },
                    "sub_problem_1": self._create_sub_problem_schema(
                        "help students understand the problem statement"
                    ),
                    "sub_problem_2": self._create_sub_problem_schema(
                        "guide students in designing an approach"
                    )
                },
                "required": ["ID", "content", "sub_problem_1", "sub_problem_2"],
                "additionalProperties": False
            }
        }

    def _create_sub_problem_schema(self, purpose: str) -> Dict[str, Any]:
        """Create schema for sub-problems."""
        return {
            "type": "object",
            "properties": {
                "description_of_problem": {
                    "type": "string",
                    "description": f"A multiple-choice question to {purpose}"
                },
                "options": {
                    "type": "array",
                    "items": {"type": "string"},
                    "minItems": 2,
                    "description": "Options should be key concepts from the content"
                },
                "answers_options": {
                    "type": "array",
                    "items": {"type": "integer"},
                    "description": "The indices of correct options"
                },
                "explanation_of_answer": {
                    "type": "string",
                    "description": "Explanation of why these options are correct"
                }
            },
            "required": [
                "description_of_problem", "options",
                "answers_options", "explanation_of_answer"
            ],
            "additionalProperties": False
        }

    def create_no_title_schema(
        self,
        paragraph_count: int,
        include_sections: bool = True
    ) -> Dict[str, Any]:
        """Create schema for content without titles."""
        schema_properties = {
            "file_description": {
                "type": "string",
                "description": "A short summary describing the file type and key content (e.g., 'A lecture recording for recursion, it mentioned base cases and tree structures')"
            },
            "paragraphs": self._create_paragraphs_schema(paragraph_count),
            "key_concepts": self._create_generic_key_concepts_schema(),
            "recap_questions": self._create_recap_question_schema(),
            "speakers": self._create_speaker_schema()
        }

        if include_sections:
            schema_properties["sections"] = self._create_sections_schema(paragraph_count)

        return {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": schema_properties,
                    "required": list(schema_properties.keys()),
                    "additionalProperties": False
                }
            }
        }

    def _create_paragraphs_schema(self, paragraph_count: int) -> Dict[str, Any]:
        """Create schema for paragraphs."""
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
                        "description": "The 1-based index from the paragraphs array",
                        "maximum": paragraph_count
                    }
                },
                "required": ["title", "paragraph_index"],
                "additionalProperties": False
            }
        }

    def _create_sections_schema(self, paragraph_count: int) -> Dict[str, Any]:
        """Create schema for sections."""
        return {
            "type": "array",
            "maxItems": self.config.max_sections,
            "items": {
                "type": "object",
                "properties": {
                    "section_title": {"type": "string"},
                    "start_paragraph_index": {
                        "type": "integer",
                        "description": "The 1-based index where this section begins",
                        "minimum": 1,
                        "maximum": paragraph_count
                    }
                },
                "required": ["section_title", "start_paragraph_index"],
                "additionalProperties": False
            }
        }

    def _create_generic_key_concepts_schema(self) -> Dict[str, Any]:
        """Create generic key concepts schema without title enum."""
        return {
            "type": "array",
            "description": "A list of key concepts extracted from the sections",
            "items": {
                "type": "object",
                "properties": {
                    "concept": {
                        "type": "string",
                        "description": "Short keyword phrase or level-1 section title"
                    },
                    "source_section_title": {
                        "type": "string",
                        "description": "The title where this concept was mentioned"
                    },
                    "aspects": self._create_aspects_schema(),
                    "check_in_question": self._create_check_in_question_schema()
                },
                "required": [
                    "concept", "source_section_title",
                    "aspects", "check_in_question"
                ],
                "additionalProperties": False
            }
        }

    def create_one_title_level_schema(self, title_list: List[str]) -> Dict[str, Any]:
        """Create schema for content with one title level."""
        return {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "file_description": {
                            "type": "string",
                            "description": "A short summary describing the file type and key content (e.g., 'A lecture recording for recursion, it mentioned base cases and tree structures')"
                        },
                        "titles_with_levels": {
                            "type": "array",
                            "description": "Titles with their hierarchical levels",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "title": {
                                        "type": "string",
                                        "enum": title_list,
                                        "description": "Exactly one title from the list"
                                    },
                                    "level_of_title": {
                                        "type": "integer",
                                        "description": "The hierarchy level (1, 2, 3, etc.)"
                                    }
                                },
                                "required": ["title", "level_of_title"],
                                "additionalProperties": False
                            }
                        },
                        "key_concepts": self._create_key_concepts_schema(title_list),
                        "recap_questions": self._create_recap_question_schema()
                    },
                    "required": ["file_description", "titles_with_levels", "key_concepts", "recap_questions"],
                    "additionalProperties": False
                }
            }
        }


# ========================
# Content Processor
# ========================

class ContentProcessor:
    """Handles content manipulation and structuring."""

    def __init__(self, config: Optional[ProcessingConfig] = None):
        """Initialize content processor with configuration."""
        self.config = config or ProcessingConfig()

    def extract_titles(self, md_content: str) -> List[str]:
        """Extract all titles from markdown content."""
        lines = md_content.split("\n")
        titles = []
        for line in lines:
            if line.startswith("#"):
                title = line.lstrip("#").strip()
                titles.append(title)
        return titles

    def count_paragraphs(self, md_text: str) -> int:
        """Count the number of paragraphs in markdown text."""
        md_text = md_text.strip()
        blocks = re.split(r'\n\s*\n', md_text)
        paragraphs = [b for b in blocks if b.strip()]
        return len(paragraphs)

    def remove_redundant_title(self, md_content: str, file_name: str) -> str:
        """Remove redundant title that matches filename."""
        normalized_filename = file_name.replace("-", " ").replace("_", " ")
        lines = md_content.split("\n")

        if not lines or not lines[0].startswith("# "):
            logger.info(f"No redundant title found in '{file_name}'")
            return md_content

        title_text = lines[0].lstrip("# ").strip()
        if normalized_filename.lower() != title_text.lower():
            logger.info(f"No redundant title found in '{file_name}'")
            return md_content

        logger.info(f"Found and removing redundant title in '{file_name}'")

        # Remove the first line and any empty lines following it
        remaining_lines = lines[1:]
        while remaining_lines and remaining_lines[0].strip() == "":
            remaining_lines.pop(0)

        # Check heading levels in remaining content
        heading_levels_found = set()
        for line in remaining_lines:
            stripped_line = line.lstrip()
            if stripped_line.startswith("#"):
                level = 0
                while level < len(stripped_line) and stripped_line[level] == "#":
                    level += 1
                heading_levels_found.add(level)

        # Promote headings if multiple levels found
        final_lines = []
        if len(heading_levels_found) > 1:
            logger.info("Multiple heading levels found. Promoting subsequent headings by one level.")
            for line in remaining_lines:
                stripped_line = line.lstrip()
                if stripped_line.startswith("##"):
                    first_hash_index = line.find("#")
                    new_line = line[:first_hash_index] + line[first_hash_index + 1:]
                    final_lines.append(new_line)
                else:
                    final_lines.append(line)
        else:
            logger.info("No promotion needed (headings are uniform or absent).")
            final_lines = remaining_lines

        return "\n".join(final_lines)

    def fix_title_levels(self, mapping_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Fix title levels in the mapping list to ensure they are sequential."""
        last_level = 0
        for i in range(len(mapping_list)):
            current_level = mapping_list[i]["level_of_title"]
            mapping_list[i]['title'] = mapping_list[i]['title'].strip()

            if current_level > last_level + 1:
                diff = current_level - (last_level + 1)
                j = i
                while j < len(mapping_list) and mapping_list[j]["level_of_title"] >= current_level:
                    mapping_list[j]["level_of_title"] -= diff
                    j += 1

            last_level = mapping_list[i]["level_of_title"]

        return mapping_list

    def apply_structure_for_no_title(
        self,
        md_content: str,
        content_dict: Dict[str, Any]
    ) -> str:
        """Apply structure to content without titles."""
        original_paragraphs = [p.strip() for p in md_content.split("\n\n") if p.strip()]
        md_parts = []
        content_dict['titles_with_levels'] = []

        # Get section starts if they exist
        section_starts = {}
        if 'sections' in content_dict:
            section_starts = {
                s["start_paragraph_index"]: s["section_title"]
                for s in content_dict.get("sections", [])
            }

        # Process paragraphs
        for paragraph in sorted(
            content_dict["paragraphs"],
            key=lambda p: p["paragraph_index"]
        ):
            p_index = paragraph["paragraph_index"]
            p_title = paragraph["title"].strip()
            level = 1

            # Add section title if this paragraph starts a new section
            if p_index in section_starts:
                title = section_starts[p_index].strip()
                md_parts.append(f"# {title}\n\n")
                content_dict['titles_with_levels'].append({
                    "title": title,
                    "level_of_title": level,
                    "paragraph_index": p_index,
                })

            # Add paragraph title
            title = p_title
            if section_starts:
                level = 2

            md_parts.append(f"{'#' * level} {title}\n\n")
            content_dict['titles_with_levels'].append({
                "title": title,
                "level_of_title": level,
                "paragraph_index": p_index,
            })

            # Add paragraph content
            content_index = p_index - 1  # Convert to 0-based index
            if content_index < len(original_paragraphs):
                md_parts.append(f"{original_paragraphs[content_index]}\n\n")

        return "".join(md_parts)

    def apply_structure_for_one_title(
        self,
        md_content: str,
        content_dict: Dict[str, Any]
    ) -> str:
        """Apply structure to content with one title level."""
        mapping_list = content_dict.get("titles_with_levels", [])
        mapping_list = self.fix_title_levels(mapping_list)

        i = 0
        lines = md_content.split("\n")
        title_pattern = re.compile(r"^(?P<hashes>#{1,6})\s+(?P<title>\S.*?)$")
        new_lines = []

        for line in lines:
            match = title_pattern.match(line.strip())
            if match and i < len(mapping_list):
                raw_title = match.group("title").strip()
                # Check if title is in mapping list
                title_variations = [
                    raw_title.replace('"', "'"),
                    raw_title
                ]

                if any(v in [d['title'] for d in mapping_list] for v in title_variations):
                    new_level = mapping_list[i]["level_of_title"]
                    new_lines.append(f"{'#' * new_level} {raw_title}")
                    i += 1
                else:
                    logger.warning(f"Title '{raw_title}' not found in mapping list, skipping")
                    continue
            else:
                new_lines.append(line)

        return "\n".join(new_lines)

    def remove_invalid_concepts(
        self,
        content_dict: Dict[str, Any],
        title_list: List[str]
    ) -> Dict[str, Any]:
        """Remove key concepts that don't have valid source_section_titles."""
        valid_concepts = []
        removed_concepts = []

        for concept in content_dict.get('key_concepts', []):
            source_title = concept.get('source_section_title', '')
            if source_title in title_list:
                valid_concepts.append(concept)
            else:
                removed_concepts.append(source_title)
                logger.info(f"Removed invalid concept with title: '{source_title}'")

        content_dict['key_concepts'] = valid_concepts

        if removed_concepts:
            logger.info(f"Total removed concepts: {len(removed_concepts)}")

        return content_dict


# ========================
# Speaker Processor
# ========================

class SpeakerProcessor:
    """Handles speaker role assignment and management."""

    def __init__(self):
        """Initialize speaker processor."""
        self.role_patterns = {
            "name_introduction": [
                r"I am (\w+)",
                r"I'm (\w+)",
                r"My name is (\w+)",
                r"This is (\w+)",
                r"I'm (?:Professor|Dr\.) (\w+)",
                r"(\w+) speaking",
                r"(\w+) here"
            ]
        }

    def assign_speaker_roles(
        self,
        md_content: str,
        speakers_mapping: List[Dict[str, str]],
        json_file_path: Optional[str] = None
    ) -> str:
        """
        Replace speaker IDs with their assigned roles in content.

        Args:
            md_content: The markdown content with speaker tags
            speakers_mapping: List of speaker mappings
            json_file_path: Optional path to JSON transcript file

        Returns:
            Updated markdown content with speaker roles replaced
        """
        if not speakers_mapping:
            logger.warning("No speakers mapping provided, returning original content")
            return md_content

        # Create mapping dictionary
        speaker_role_map = {
            speaker["speaker_id"]: speaker["role"]
            for speaker in speakers_mapping
        }

        updated_content = md_content

        # Replace speaker IDs in markdown
        for speaker_id, role in speaker_role_map.items():
            # Replace patterns like "Speaker_00:" with role
            pattern = rf'\b{re.escape(speaker_id)}:'
            replacement = f'{role}:'
            updated_content = re.sub(pattern, replacement, updated_content)

            # Also replace bold patterns
            pattern_bold = rf'\*\*{re.escape(speaker_id)}:\*\*'
            replacement_bold = f'**{role}:**'
            updated_content = re.sub(pattern_bold, replacement_bold, updated_content)

        # Update JSON file if provided
        if json_file_path and os.path.exists(json_file_path):
            self._update_speakers_in_json(json_file_path, speaker_role_map)

        logger.info(f"Updated speakers: {', '.join([f'{k} -> {v}' for k, v in speaker_role_map.items()])}")
        return updated_content

    def _update_speakers_in_json(
        self,
        json_file_path: str,
        speaker_role_map: Dict[str, str]
    ) -> None:
        """Update speaker IDs in JSON transcript file."""
        try:
            with open(json_file_path, "r") as json_file:
                transcript_data = json.load(json_file)

            updated_count = 0
            for entry in transcript_data:
                if "speaker" in entry and entry["speaker"] in speaker_role_map:
                    old_speaker = entry["speaker"]
                    entry["speaker"] = speaker_role_map[old_speaker]
                    updated_count += 1

            with open(json_file_path, "w") as json_file:
                json.dump(transcript_data, json_file, indent=4)

            logger.info(f"Updated {updated_count} speaker entries in JSON file: {json_file_path}")

        except Exception as e:
            logger.error(f"Error updating speakers in JSON file {json_file_path}: {str(e)}")

    def extract_and_assign_speakers(
        self,
        content_dict: Dict[str, Any],
        md_content: str,
        json_file_path: Optional[str] = None
    ) -> str:
        """
        Extract speakers from analysis and assign roles.

        Args:
            content_dict: The structured content dictionary from analysis
            md_content: The original markdown content
            json_file_path: Optional path to JSON transcript file

        Returns:
            Updated markdown content with speaker roles assigned
        """
        speakers_mapping = content_dict.get("speakers", [])

        if not speakers_mapping:
            logger.warning("No speakers found in content analysis")
            return md_content

        return self.assign_speaker_roles(md_content, speakers_mapping, json_file_path)


# ========================
# Transcript Manager
# ========================

class TranscriptManager:
    """Manages transcript-related operations."""

    def __init__(self, config: Optional[ProcessingConfig] = None):
        """Initialize transcript manager with configuration."""
        self.config = config or ProcessingConfig()

    def add_titles_to_transcript(
        self,
        json_file_path: str,
        index_helper: Dict[str, List[float]]
    ) -> None:
        """
        Insert title entries into a transcript based on timing.

        Args:
            json_file_path: Path to the JSON file containing transcript
            index_helper: Dictionary with title as key and start time as value
        """
        with open(json_file_path, "r") as json_file:
            transcript_list = json.load(json_file)

        # Convert index_helper to list of tuples
        titles_to_insert = [
            (title, start_time[0])
            for title, start_time in index_helper.items()
        ]
        titles_to_insert.sort(key=lambda x: float(x[1]))

        current_index = 0
        for title, start_time in titles_to_insert:
            # Find insertion position
            while (current_index < len(transcript_list) and
                   float(transcript_list[current_index]["start time"]) < float(start_time)):
                current_index += 1

            # Create title entry
            title_entry = {
                "start time": float(start_time),
                "end time": float(start_time),
                "speaker": f"title-{len(title)}",
                "text content": f"{title[-1]}"
            }

            transcript_list.insert(current_index, title_entry)
            current_index += 1

        # Save updated transcript
        with open(json_file_path, "w") as json_file:
            json.dump(transcript_list, json_file, indent=4)

    def group_sentences_in_transcript(
        self,
        json_file_path: str,
        output_path: Optional[str] = None
    ) -> str:
        """
        Group sentences in transcript based on speaker and time rules.

        Args:
            json_file_path: Path to the JSON transcript file
            output_path: Optional output path for grouped JSON

        Returns:
            Path to the output file
        """
        try:
            with open(json_file_path, "r") as json_file:
                transcript_data = json.load(json_file)

            if not transcript_data:
                logger.warning(f"Empty transcript data in {json_file_path}")
                return json_file_path

            grouped_transcript = []
            current_group = None

            for entry in transcript_data:
                # Null-safety: Skip entries with invalid timestamps
                start_time_raw = entry.get("start time")
                end_time_raw = entry.get("end time")

                if start_time_raw is None or end_time_raw is None:
                    logger.warning(
                        f"Skipping transcript entry with null timestamp: "
                        f"start={start_time_raw}, end={end_time_raw}, "
                        f"speaker={entry.get('speaker', 'N/A')}"
                    )
                    continue

                try:
                    start_time = float(start_time_raw)
                    end_time = float(end_time_raw)
                except (TypeError, ValueError) as e:
                    logger.warning(
                        f"Skipping transcript entry with invalid timestamp: {e}, "
                        f"start={start_time_raw}, end={end_time_raw}"
                    )
                    continue

                speaker = entry.get("speaker", "")
                text_content = entry.get("text content", "").strip()

                if not text_content:
                    continue

                # Check if entry is a title
                is_title = speaker.startswith("title-")

                # Determine if we can add to current group
                if current_group is None:
                    current_group = self._create_group_entry(
                        start_time, end_time, speaker, text_content
                    )
                elif self._should_start_new_group(
                    current_group, entry, is_title
                ):
                    grouped_transcript.append(current_group)
                    current_group = self._create_group_entry(
                        start_time, end_time, speaker, text_content
                    )
                else:
                    # Add to current group
                    combined_text = f"{current_group['text content']} {text_content}"
                    current_group["text content"] = combined_text
                    current_group["end time"] = end_time

            # Add the last group
            if current_group is not None:
                grouped_transcript.append(current_group)

            # Write output
            output_file_path = output_path if output_path else json_file_path
            with open(output_file_path, "w") as json_file:
                json.dump(grouped_transcript, json_file, indent=4)

            # Log statistics
            original_count = len(transcript_data)
            grouped_count = len(grouped_transcript)
            reduction_percent = ((original_count - grouped_count) / original_count * 100) if original_count > 0 else 0

            logger.info(
                f"Grouped transcript sentences: {original_count} -> {grouped_count} "
                f"({reduction_percent:.1f}% reduction) in {output_file_path}"
            )

            return output_file_path

        except Exception as e:
            logger.error(f"Error grouping sentences in transcript {json_file_path}: {str(e)}")
            return json_file_path

    def _create_group_entry(
        self,
        start_time: float,
        end_time: float,
        speaker: str,
        text_content: str
    ) -> Dict[str, Any]:
        """Create a new group entry."""
        return {
            "start time": start_time,
            "end time": end_time,
            "speaker": speaker,
            "text content": text_content
        }

    def _should_start_new_group(
        self,
        current_group: Dict[str, Any],
        entry: Dict[str, Any],
        is_title: bool
    ) -> bool:
        """Determine if a new group should be started."""
        # Titles should never be grouped
        if is_title or current_group.get("speaker", "").startswith("title-"):
            return True

        # Different speaker
        if current_group["speaker"] != entry.get("speaker", ""):
            return True

        # Time gap too large
        start_time = float(entry.get("start time", 0))
        if (start_time - float(current_group["end time"])) > self.config.max_time_gap_seconds:
            return True

        # Word limit check
        combined_text = f"{current_group['text content']} {entry.get('text content', '')}"
        word_count = len(combined_text.split())
        if word_count > self.config.max_words_per_group:
            return True

        return False


# ========================
# Prompt Builder
# ========================

class PromptBuilder:
    """Builds prompts for OpenAI API calls."""

    def __init__(self, config: Optional[ProcessingConfig] = None):
        """Initialize prompt builder with configuration."""
        self.config = config or ProcessingConfig()

    def build_ipynb_prompt(
        self,
        course_name: str,
        file_name: str
    ) -> str:
        """Build prompt for ipynb content processing."""
        return dedent(
            f"""You are an expert AI assistant specializing in analyzing and structuring educational material.
            You will be given markdown content from a video in the course "{course_name}", from the file "{file_name}".
            Your task is to perform the following actions and format the output as a single JSON object:

            ### Part 0: Generate File Description
            Provide a 1-2 sentence summary describing what type of file this is and what key topics/concepts it covers.
            Example: "A lecture recording for recursion, it mentioned base cases, recursive calls, and tree structures."

            ### Part 1: Extract Key Concepts
            Your goal is to create a high-level summary of the entire document by identifying a small, curated set of its most important concepts.

            CRITICAL CONSTRAINTS - YOU MUST FOLLOW:
            1. **Strict One-to-One Mapping:** Each Source Section title MUST map to exactly ONE Key Concept.
            2. **Limited Quantity:** Aggressively merge and consolidate topics. The final count must always be less than {self.config.max_key_concepts}.
            3. **No Hierarchical Overlap:** Cannot choose both a main section and its sub-section.
            4. **Concise Concepts:** Short keyword phrases or level-1 section titles. NOT full sentences.
            5. **Analyze with Aspects:** For each key concept, provide multiple detailed analytical descriptions from different perspectives (e.g., definition, examples, use cases, implications). Each aspect should be a longer description analyzing the concept thoroughly.
            6. **Preserve Original Order:** Maintain the sequence as in the original markdown.
            7. **Generate Follow-up Check-in Question:** Create challenging questions requiring application/analysis/synthesis.

            ### Part 2: Extract and Create Problems
            If you find blocks named Exercise or Challenge, create educational problems that test understanding.
            For each problem:
            - **ID:** Identify if this relates to an existing exercise
            - **Content:** Provide the main problem statement
            - **Sub-problems:** Create exactly 2 sub-problems as multiple choice questions

            ### Part 3: Generate Comprehensive Recap Questions
            Create up to {self.config.max_recap_questions} recap questions (0–{self.config.max_recap_questions}, only meaningful ones).

            Design principles:
            - **Recap, not add-on:** Use document's terminology and scope
            - **Coverage with minimal redundancy:** Touch main themes without overlap
            - **Easy → Moderate:** Order from easier to slightly more integrative
            - **Concept integration:** Connect 2–3 core ideas when reasonable

            Format your response as a valid JSON object matching the provided schema."""
        )

    def build_no_title_prompt(
        self,
        course_name: str,
        file_name: str,
        paragraph_count: int
    ) -> str:
        """Build prompt for content without titles."""
        if paragraph_count > self.config.min_paragraphs_for_sections:
            return self._build_multi_paragraph_prompt(course_name, file_name)
        else:
            return self._build_few_paragraph_prompt(course_name, file_name)

    def _build_multi_paragraph_prompt(
        self,
        course_name: str,
        file_name: str
    ) -> str:
        """Build prompt for content with multiple paragraphs."""
        return dedent(
            f"""You are an expert AI assistant specializing in analyzing and structuring educational material.
            You will be given markdown content from a video in the course "{course_name}", from the file "{file_name}".

            ### Part 0: Generate File Description
            Provide a 1-2 sentence summary describing what type of file this is and what key topics/concepts it covers.
            Example: "A lecture recording for recursion, it mentioned base cases, recursive calls, and tree structures."

            ### Part 1: Structure the Content
            1. **Group into Sections:** Divide the text into 3-{self.config.max_sections} logical sections.
            2. **Generate Titles:** Create concise, descriptive titles for sections and paragraphs.
            3. **Create Nested Structure:** Organize with section titles and nested paragraph arrays.

            ### Part 2: Extract Key Concepts
            Based on sections from Part 1, distill core learning points with assessment questions.

            CRITICAL CONSTRAINTS:
            - **Concise Concepts:** Short keyword phrases or level-1 section titles. NOT full sentences.
            - **Analyze with Aspects:** For each key concept, provide multiple detailed analytical descriptions from different perspectives (e.g., definition, examples, use cases, implications). Each aspect should be a longer description analyzing the concept thoroughly.
            - Follow the same constraints as the standard key concept extraction.

            ### Part 3: Generate Recap Questions
            Create up to {self.config.max_recap_questions} meaningful recap questions for review.

            ### Part 4: Identify and Classify Speakers
            Analyze speaker tags (Speaker_00, etc.) and determine roles:
            - Look for name introductions first
            - If no names, classify as Professor (only one), TA_N, Student_N, Unknown_N

            Format your response as a valid JSON object matching the provided schema."""
        )

    def _build_few_paragraph_prompt(
        self,
        course_name: str,
        file_name: str
    ) -> str:
        """Build prompt for content with few paragraphs."""
        return dedent(
            f"""You are an expert AI assistant specializing in analyzing and structuring educational material.
            You will be given markdown content from a video in the course "{course_name}", from the file "{file_name}".

            ### Part 0: Generate File Description
            Provide a 1-2 sentence summary describing what type of file this is and what key topics/concepts it covers.
            Example: "A lecture recording for recursion, it mentioned base cases, recursive calls, and tree structures."

            ### Part 1: Structure the Content
            **Generate Titles:** Create one concise, descriptive title for each paragraph.

            ### Part 2: Extract Key Concepts
            Distill core learning points from paragraphs with assessment questions.

            CRITICAL CONSTRAINTS:
            - **Concise Concepts:** Short keyword phrases or level-1 section titles (e.g., 'conda activate yk_env', 'recursion tree'). NOT full sentences.
            - **Analyze with Aspects:** For each key concept, provide multiple detailed analytical descriptions from different perspectives (e.g., definition, examples, use cases, implications). Each aspect should be a longer description analyzing the concept thoroughly.
            - Maximum {self.config.max_key_concepts} concepts, following standard constraints.

            ### Part 3: Generate Recap Questions
            Create up to {self.config.max_recap_questions} meaningful recap questions.

            ### Part 4: Identify and Classify Speakers
            Analyze and classify speakers, prioritizing actual names if mentioned.

            Format your response as a valid JSON object matching the provided schema."""
        )

    def build_one_title_level_prompt(
        self,
        course_name: str,
        file_name: str,
        title_list: List[str],
    ) -> str:
        """Build prompt for content with one title level."""
        return dedent(
            f"""You are an expert AI assistant for structuring educational material.
            You will be given markdown content from the file "{file_name}" for the course "{course_name}".

            ### Part 0: Generate File Description
            Provide a 1-2 sentence summary describing what type of file this is and what key topics/concepts it covers.
            Example: "A lecture recording for recursion, it mentioned base cases, recursive calls, and tree structures."

            ### Part 1: Adjust Title Hierarchy Levels
            Given the title list, determine correct semantic hierarchy levels based on logical relationships.

            CRITICAL CONSTRAINTS:
            - Use EXACTLY the titles from title_list - no modifications
            - Output title text WITHOUT any '#' symbols
            - Do NOT create new titles or modify existing ones
            - Maintain logical flow relative to surrounding titles

            ### Part 2: Extract High-Level Key Concepts
            Create a high-level summary with {self.config.max_key_concepts} or fewer key concepts.

            CRITICAL CONSTRAINTS:
            - **Concise Concepts:** Short keyword phrases or level-1 section titles. NOT full sentences.
            - **Analyze with Aspects:** For each key concept, provide multiple detailed analytical descriptions from different perspectives (e.g., definition, examples, use cases, implications). Each aspect should be a longer description analyzing the concept thoroughly.
            - Follow standard key concept extraction constraints.

            ### Part 3: Generate Recap Questions
            Create up to {self.config.max_recap_questions} meaningful recap questions.

            Format your response as a valid JSON object matching the provided schema."""
        )

    def build_key_concepts_only_prompt(self) -> str:
        """Build prompt for extracting only key concepts."""
        return dedent(
            f"""Extract High-Level Key Concepts for Overview
            Your goal is to create a high-level summary by identifying the most important concepts.

            ### Generate File Description
            Provide a 1-2 sentence summary describing what type of file this is and what key topics/concepts it covers.
            Example: "A lecture recording for recursion, it mentioned base cases, recursive calls, and tree structures."

            CRITICAL CONSTRAINTS:
            1. **Strict One-to-One Mapping:** Each Source Section maps to exactly ONE Key Concept
            2. **Limited Quantity:** Maximum {self.config.max_key_concepts} concepts
            3. **No Hierarchical Overlap:** Cannot choose both main and sub-sections
            4. **Concise Concepts:** Short keyword phrases or level-1 section titles. NOT full sentences.
            5. **Analyze with Aspects:** For each key concept, provide multiple detailed analytical descriptions from different perspectives (e.g., definition, examples, use cases, implications). Each aspect should be a longer description analyzing the concept thoroughly.
            6. **Preserve Original Order:** Maintain source document sequence
            7. **Generate Check-in Questions:** Create challenging assessment questions

            ### Generate Comprehensive Recap Questions
            Create up to {self.config.max_recap_questions} meaningful recap questions for review.

            Design principles:
            - **Recap, not add-on:** Use document's terminology
            - **Coverage with minimal redundancy:** Touch main themes
            - **Easy → Moderate:** Order by difficulty
            - **Concept integration:** Connect 2-3 core ideas

            Format your response as a valid JSON object matching the provided schema."""
        )


# ========================
# Title Handler (Main Orchestrator)
# ========================

class TitleHandler:
    """Main orchestrator for title and content handling."""

    def __init__(self, api_key: Optional[str] = None, use_openai: bool = True):
        """Initialize title handler with all components.

        Args:
            api_key: Optional OpenAI API key
            use_openai: Whether to use OpenAI (default True for backward compatibility)
        """
        self.config = ProcessingConfig()
        self.use_openai = use_openai

        # Only initialize OpenAI if needed
        if use_openai:
            self.openai_client = OpenAIClientWrapper(api_key, lazy_init=True)
        else:
            self.openai_client = None

        self.schema_factory = SchemaFactory(self.config)
        self.content_processor = ContentProcessor(self.config)
        self.speaker_processor = SpeakerProcessor()
        self.transcript_manager = TranscriptManager(self.config)
        self.prompt_builder = PromptBuilder(self.config)

    def process_ipynb_content(
        self,
        md_content: str,
        file_name: str,
        course_name: str,
        index_helper: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Process ipynb content and extract structured information."""
        if not md_content.strip():
            raise ValueError("The content is empty or not properly formatted.")

        # Extract title list
        title_list = self._prepare_title_list(index_helper)

        # Build prompt and schema
        prompt = self.prompt_builder.build_ipynb_prompt(course_name, file_name)
        schema = self.schema_factory.create_ipynb_schema(title_list)

        # Get response from OpenAI
        content_dict = self.openai_client.create_completion(prompt, md_content, schema)

        # Clean up invalid concepts
        content_dict = self.content_processor.remove_invalid_concepts(content_dict, title_list)

        return content_dict

    def process_content_without_titles(
        self,
        md_content: str,
        file_name: str,
        course_name: str
    ) -> Dict[str, Any]:
        """Process content that doesn't have titles."""
        if not md_content.strip():
            raise ValueError("The content is empty or not properly formatted.")

        paragraph_count = self.content_processor.count_paragraphs(md_content)

        # Build prompt and schema
        prompt = self.prompt_builder.build_no_title_prompt(
            course_name, file_name, paragraph_count
        )
        include_sections = paragraph_count > self.config.min_paragraphs_for_sections
        schema = self.schema_factory.create_no_title_schema(
            paragraph_count, include_sections
        )

        # Get response from OpenAI
        content_dict = self.openai_client.create_completion(prompt, md_content, schema)

        return content_dict

    def process_content_with_one_title_level(
        self,
        md_content: str,
        file_name: str,
        course_name: str,
        index_helper: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Process content with one title level."""
        if not md_content.strip():
            raise ValueError("The content is empty or not properly formatted.")

        # Process content
        md_content = self.content_processor.remove_redundant_title(md_content, file_name)
        title_list = self._prepare_title_list(index_helper)

        # Check for single title, single paragraph case
        paragraph_count = self.content_processor.count_paragraphs(md_content)
        if len(title_list) == 1 and paragraph_count == 1:
            return {
                "titles_with_levels": [{
                    "title": title_list[0],
                    "level_of_title": 1,
                    "paragraph_index": 1
                }],
                "key_concepts": []
            }

        # Build prompt and schema
        prompt = self.prompt_builder.build_one_title_level_prompt(
            course_name, file_name, title_list
        )
        schema = self.schema_factory.create_one_title_level_schema(title_list)

        # Get response from OpenAI
        content_dict = self.openai_client.create_completion(prompt, md_content, schema)

        return content_dict

    def extract_key_concepts_only(
        self,
        md_content: str,
        index_helper: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Extract only key concepts from content."""
        title_list = self._prepare_title_list(index_helper) or self.content_processor.extract_titles(md_content)

        # Check if OpenAI is available
        if not self.use_openai or self.openai_client is None:
            logger.warning("OpenAI is not available. Returning empty key concepts structure.")
            return {
                "file_description": "",
                "titles_with_levels": [{"title": title, "level_of_title": 1} for title in title_list],
                "key_concepts": [],
                "recap_questions": []
            }

        # Build prompt and schema
        prompt = self.prompt_builder.build_key_concepts_only_prompt()
        schema = self.schema_factory.create_one_title_level_schema(title_list)

        # Get response from OpenAI
        try:
            content_dict = self.openai_client.create_completion(prompt, md_content, schema)
        except ValueError as e:
            logger.warning(f"OpenAI API error: {e}. Returning empty key concepts structure.")
            return {
                "file_description": "",
                "titles_with_levels": [{"title": title, "level_of_title": 1} for title in title_list],
                "key_concepts": [],
                "recap_questions": []
            }

        return content_dict

    def _prepare_title_list(self, index_helper: Optional[Dict[str, Any]]) -> List[str]:
        """Prepare title list from index helper."""
        if not index_helper:
            return []

        return [
            key.replace('"', "'")
            for d in index_helper
            for key in d.keys()
        ]

    def save_key_concepts_to_metadata(
        self,
        json_dict: Dict[str, Any],
        metadata_path: Path
    ) -> None:
        """Save key concepts to metadata file."""
        # Load existing metadata or create new
        if metadata_path.exists():
            with open(metadata_path, "r") as metadata_file:
                data = yaml.safe_load(metadata_file) or {}
        else:
            logger.warning("No metadata file exists, creating a new one.")
            metadata_path.parent.mkdir(parents=True, exist_ok=True)
            data = {}

        logger.info(f"Creating/updating metadata file at: {metadata_path}")

        # Update key concepts
        key_concepts = json_dict.get("key_concepts", [])
        if key_concepts:
            key_concepts[0]["source_section_title"] = key_concepts[0]["source_section_title"].strip()
            data["key_concept"] = key_concepts

        # Save metadata
        with open(metadata_path, "w") as metadata_file:
            yaml.safe_dump(data, metadata_file, default_flow_style=False)


# ========================
# Public API Functions
# ========================

def get_strutured_content_for_ipynb(
    md_content: str,
    file_name: str,
    course_name: str,
    index_helper: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Process markdown content from ipynb to extract structured information.

    This is the main entry point for processing Jupyter notebook content.
    """
    handler = TitleHandler()
    # First get titles for validation
    _ = handler.content_processor.extract_titles(md_content)
    return handler.process_ipynb_content(md_content, file_name, course_name, index_helper)


def get_structured_content_without_title(
    md_content: str,
    file_name: str,
    course_name: str
) -> Dict[str, Any]:
    """
    Process markdown content without titles to generate structure.

    This function handles content that lacks title structure.
    """
    handler = TitleHandler()
    return handler.process_content_without_titles(md_content, file_name, course_name)


def get_structured_content_with_one_title_level(
    md_content: str,
    file_name: str,
    course_name: str,
    index_helper: Dict[str, Any]
) -> Dict[str, Any]:
    """
    Process markdown content with a single title level.

    This function handles content with flat title structure.
    """
    handler = TitleHandler()
    return handler.process_content_with_one_title_level(
        md_content, file_name, course_name, index_helper
    )


def get_only_key_concepts(
    md_content: str,
    index_helper: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Extract only key concepts from markdown content.

    This is a simplified function that focuses on key concept extraction.
    If OpenAI API key is not available, returns empty structure.
    """
    # Check if OpenAI API key is available using stable function
    api_key = get_openai_api_key()
    use_openai = api_key is not None

    handler = TitleHandler(use_openai=use_openai)
    return handler.extract_key_concepts_only(md_content, index_helper)


def get_title_list(md_content: str) -> List[str]:
    """
    Extract all titles from markdown content.

    This is a utility function for title extraction.
    """
    processor = ContentProcessor()
    return processor.extract_titles(md_content)


def remove_redundant_title(md_content: str, file_name: str) -> str:
    """
    Remove redundant title that matches the filename.

    This is a utility function for content cleanup.
    """
    processor = ContentProcessor()
    return processor.remove_redundant_title(md_content, file_name)


def remove_invalid_concepts(
    content_dict: Dict[str, Any],
    title_list: List[str]
) -> Dict[str, Any]:
    """
    Remove key concepts with invalid source section titles.

    This is a utility function for data validation.
    """
    processor = ContentProcessor()
    return processor.remove_invalid_concepts(content_dict, title_list)


def fix_title_levels(mapping_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Fix title levels to ensure they are sequential.

    This is a utility function for title hierarchy correction.
    """
    processor = ContentProcessor()
    return processor.fix_title_levels(mapping_list)


def apply_structure_for_no_title(
    md_content: str,
    content_dict: Dict[str, Any]
) -> str:
    """
    Apply structure to content without existing titles.

    This function adds titles to unstructured content.
    """
    processor = ContentProcessor()
    return processor.apply_structure_for_no_title(md_content, content_dict)


def apply_structure_for_one_title(
    md_content: str,
    content_dict: Dict[str, Any]
) -> str:
    """
    Apply structure to content with one title level.

    This function restructures flat title hierarchies.
    """
    processor = ContentProcessor()
    return processor.apply_structure_for_one_title(md_content, content_dict)


def save_key_concept_to_metadata(
    json_dict: Dict[str, Any],
    metadata_path: Path
) -> None:
    """
    Save key concepts to a metadata YAML file.

    This function persists key concepts for later use.
    """
    handler = TitleHandler()
    handler.save_key_concepts_to_metadata(json_dict, metadata_path)


def assign_speaker_roles_to_content(
    md_content: str,
    speakers_mapping: List[Dict[str, str]],
    json_file_path: Optional[str] = None
) -> str:
    """
    Replace speaker IDs with their assigned roles in content.

    This function updates speaker identifiers with meaningful roles.
    """
    processor = SpeakerProcessor()
    return processor.assign_speaker_roles(md_content, speakers_mapping, json_file_path)


def extract_and_assign_speakers(
    content_dict: Dict[str, Any],
    md_content: str,
    json_file_path: Optional[str] = None
) -> str:
    """
    Extract and assign speaker roles from content analysis.

    This is a convenience function for speaker management.
    """
    processor = SpeakerProcessor()
    return processor.extract_and_assign_speakers(content_dict, md_content, json_file_path)


def add_titles_to_json(
    index_helper: Dict[str, List[float]],
    json_file_path: str
) -> None:
    """
    Add title entries to a transcript JSON file.

    This function inserts titles at appropriate timestamps.
    """
    manager = TranscriptManager()
    manager.add_titles_to_transcript(json_file_path, index_helper)


def group_sentences_in_transcript(
    json_file_path: str,
    max_time_gap: float = 5.0,
    max_words: int = 200,
    output_path: Optional[str] = None
) -> str:
    """
    Group sentences in transcript based on speaker and timing rules.

    This function consolidates transcript entries for better readability.
    """
    config = ProcessingConfig()
    config.max_time_gap_seconds = max_time_gap
    config.max_words_per_group = max_words
    manager = TranscriptManager(config)
    return manager.group_sentences_in_transcript(json_file_path, output_path)


def update_speakers_in_json_file(
    json_file_path: str,
    speaker_role_map: Dict[str, str]
) -> None:
    """
    Update speaker IDs in JSON transcript file.

    This is a utility function for JSON transcript updates.
    """
    processor = SpeakerProcessor()
    processor._update_speakers_in_json(json_file_path, speaker_role_map)


# Utility functions for backwards compatibility
def generate_json_schema_for_no_title(
    paragraph_count: int,
    course_name: str,
    file_name: str
) -> Tuple[str, Dict[str, Any]]:
    """
    Generate prompt and schema for content without titles.

    This function is maintained for backwards compatibility.
    """
    config = ProcessingConfig()
    prompt_builder = PromptBuilder(config)
    schema_factory = SchemaFactory(config)

    prompt = prompt_builder.build_no_title_prompt(course_name, file_name, paragraph_count)
    include_sections = paragraph_count > config.min_paragraphs_for_sections
    schema = schema_factory.create_no_title_schema(paragraph_count, include_sections)

    return prompt, schema


def paragraph_count(md_text: str) -> int:
    """
    Count paragraphs in markdown text.

    This is a utility function maintained for backwards compatibility.
    """
    processor = ContentProcessor()
    return processor.count_paragraphs(md_text)


# Helper functions for transcript processing
def find_insertion_position(transcript_list: List[Dict[str, Any]], target_time: str) -> int:
    """Find where to insert a title based on its start time."""
    target_seconds = float(target_time)

    for i, entry in enumerate(transcript_list):
        entry_start_seconds = float(entry["start time"])
        if entry_start_seconds >= target_seconds:
            return i

    return len(transcript_list)


def get_previous_end_time(transcript_list: List[Dict[str, Any]], position: int) -> float:
    """Get the end time of the previous entry."""
    if position == 0:
        return 0.0
    return transcript_list[position - 1]["end time"]


def get_next_start_time(transcript_list: List[Dict[str, Any]], position: int) -> float:
    """Get the start time of the next entry."""
    if position >= len(transcript_list):
        return get_previous_end_time(transcript_list, position)
    return transcript_list[position]["start time"]