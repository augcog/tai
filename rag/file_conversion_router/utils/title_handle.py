from dotenv import load_dotenv
import os
from textwrap import dedent
from openai import OpenAI
from pathlib import Path
import yaml
import json
import re
from loguru import logger


def get_strutured_content_for_ipynb(
        md_content: str, file_name: str, course_name: str,index_helper: dict = None
):
    """
    This function processes markdown content from ipynb to extract key concepts and problems for educational material.
    It uses the OpenAI API to analyze the content and generate a structured JSON output.
    Args:
        md_content (str): The markdown content to be processed.
        file_name (str): The name of the file containing the markdown content.
        course_name (str): The name of the course related to the content.
    """
    get_title_list(md_content)
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)
    if not md_content.strip():
        raise ValueError("The content is empty or not properly formatted.")
    title_list = [key.replace('"',"'") for d in index_helper for key in d.keys()]
    response = client.chat.completions.create(
        model="gpt-4.1",
        temperature=0.1,
        messages=[
            {
                "role": "system",
                "content": dedent(
                    f"""You are an expert AI assistant specializing in analyzing and structuring educational material. 
                    You will be given markdown content from a video in the course "{course_name}", from the file "{file_name}". 
                    Your task is to perform the following actions and format the output as a single JSON object: 
                    ### Part 1: Extract Key Concepts 
                    Your goal is to create a high-level summary of the entire document by identifying a small, curated set of its most important concepts. This should serve as a "big picture" overview for a student.
                    CRITICAL CONSTRAINTS - YOU MUST FOLLOW:
                    1.  **Strict One-to-One Mapping (Most Important):** The relationship between a Source Section and a Key Concept must be strictly one-to-one. Each chosen Source Section title MUST map to exactly ONE Key Concept. It is forbidden to generate multiple Key Concepts from the same single Source Section.
                    2.  **Limited Quantity:** Your most important task is to aggressively merge and consolidate topics. Actively seek to unify related ideas under a single, overarching key concept. Before creating a new concept, you must first determine if its core idea can be logically absorbed by another. The final output must represent the absolute minimum number of concepts possible, and the count must always be less than 5.
                    3.  **No Hierarchical Overlap:** If you choose a main section title (e.g., "Chapter 1"), you cannot also choose one of its sub-sections (e.g., "Section 1.1").
                    4.  **Concise Concepts:** The key concept should be a single, descriptive sentence that captures the main idea of the entire section block.
                    5.  **Preserve Original Order:** The Key Concept objects must appear in the same order as their corresponding Source Sections appear in the original markdown. Do not reorder, merge across, or reshuffle the sequence.
                    6.  **Generate Follow-up Assessment Question:** For each key concept, you must create an `assessment_question` object. This object is designed to test a student's deep understanding of the concept.
                        -   **Challenging Nature:** The question must be difficult. It should require **application, analysis, or synthesis** of the information from the section, not just simple recall.
                        -   **Plausible Distractors:** The incorrect options should be plausible and target common student misconceptions related to the topic.
                        -   **Structure:** The `assessment_question` object must contain the following four fields:
                            -   `question_text`: (String) The full text of the multiple-choice question.
                            -   `options`: (Array of Strings) An array containing exactly four possible answers.
                            -   `correct_answer`: (Array of Integers) The indices of the correct options in the options array, e.g. [0,1] for the first and second options.
                            -   `explanation`: (String) A detailed explanation describing why the correct answer is right and why the other options are incorrect.
                    For each concept you extract, provide ONLY the following information:
                    Key Concept: [A single, clear sentence summarizing the core idea of the section.]
                    Source Section: [The single, exact title from title_list that this concept is derived from.]
                    Assessment Question: The structured question object as defined above.

                    ### Part 2: Extract and Create Problems
                    If you find there are some blocks named Exercise or Challenge, then based on the key concepts you identified, create educational problems that test student understanding. 
                    If there are no such blocks, skip this part.
                    For each problem:
                    - **ID:** Identify if this relates to an existing exercise in the material (e.g., "Exercise 1", "Challenge 1")
                    - **Content:** Provide the main problem statement or scenario
                    - **Sub-problems:** Create exactly 2 sub-problems for each main problem:
                      - Each sub-problem should be a multiple choice question
                      - Options should include relevant key concepts from the material
                      - Provide clear explanations for correct answers
                      - Sub-problems should test different aspects or depths of understanding of the main concept

                    ### Guidelines:
                    - Focus on the most important concepts that students need to understand
                    - Ensure all content is directly derived from the provided markdown material
                    - Make problems challenging but fair, testing true comprehension
                    - Use key concepts as both correct answers and plausible distractors in multiple choice options
                    - Provide detailed explanations that reference the source material

                    Format your response as a valid JSON object matching the provided schema."""
                ),
            },
            {"role": "user", "content": f"{md_content}"},
        ],
        response_format={
            'type': 'json_schema',
            'json_schema': {
                'name': 'course_content_knowledge_sorting',
                'strict': True,
                'schema': {
                    'type': 'object',
                    'properties': {
                        'key_concepts': {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "concepts": {"type": "string"},
                                    "source_section_title": {"type": "string",
                                                             "enum": title_list,
                                                             "description": f"Exactly the section title as it appears in the {title_list}, only one title from the list, only start with # can be used, do not include # and any *. Do not treat the line start with * as a title, it is not a title."},
                                    "assessment_question": {
                                        "type": "object",
                                        "properties": {
                                            "question_text": {"type": "string", "description": "The full text of the multiple-choice question."},
                                            "options": {"type": "array", "items": {"type": "string"}, "description": "An array containing exactly four possible answers."},
                                            "correct_answer": {"type": "array", "items": {"type": "integer"}, "description": "The indices of the correct options in the options array, e.g. [0,1] for the first and second options."},
                                            "explanation": {"type": "string", "description": "A detailed explanation describing why the correct answer is right and why the other options are incorrect."}
                                        },
                                        "required": ["question_text", "options", "correct_answer", "explanation"],
                                        "additionalProperties": False
                                    },
                                    "content_coverage": {
                                        "type": "array",
                                        "description": "List only the aspects that the section actually explained with aspect and content.",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string",
                                                           'minLength': 1,},
                                                "content": {"type": "string"},
                                            },
                                            "required": ["aspect", "content"],
                                            "additionalProperties": False,
                                        },
                                    },
                                },
                                "required": [
                                    "concepts",
                                    "source_section_title",
                                    "assessment_question",
                                    "content_coverage",
                                ],
                                "additionalProperties": False,
                            },
                        },
                        'problems': {
                            "type": "array",
                            'items': {
                                "type": "object",
                                "properties": {
                                    "ID": {
                                        "type": "string",
                                        'description': 'Which Exercise id this problem belongs to, e.g. "Exercise 1","Challenge 1"'
                                    },
                                    'content': {
                                        'type': "string",
                                        'description': 'The whole content of Exercise or Challenge. Do not miss or change any part of the description.'
                                    },
                                    "sub_problem_1": {
                                        'type': "object",
                                        'properties': {
                                            'description_of_problem': {
                                                'type': 'string',
                                                'description': 'A multiple-choice question that better contains at least two correct options. Its purpose is to help students understand the problem statement and identify the underlying concepts. e.g. “What knowledge are involved in this problem?”'
                                            },
                                            'options': {
                                                'type': 'array',
                                                'items': {'type': 'string'},
                                                'description': 'The options for the sub problem, they should be key_concepts in the md_content'
                                            },
                                            'answers_options': {
                                                'type': 'array',
                                                'items': {'type': 'integer'},
                                                'description': 'The index of the option in the options array, e.g. [0,1] for the first and second options'
                                            },
                                            'explanation_of_answer': {
                                                'type': 'string',
                                                'description': 'The explanation of why this option is the answer, it should be a key_concept in the md_content'
                                            },
                                        },
                                        'required': ['description_of_problem', 'options', 'answers_options',
                                                     'explanation_of_answer'],
                                        'additionalProperties': False,
                                    },
                                    "sub_problem_2": {
                                        'type': "object",
                                        'properties': {
                                            'description_of_problem': {
                                                'type': 'string',
                                                'description': 'A multiple-choice question that better contain at least two correct options. Its purpose is to guide students in designing an approach or solution framework. e.g. “Which methods could be applied to solve this problem effectively?”'
                                            },
                                            'options': {
                                                'type': 'array',
                                                'items': {'type': 'string'},
                                                'description': 'The options for the sub problem, they should be key_concepts in the md_content'
                                            },
                                            'answers_options': {
                                                'type': 'array',
                                                'items': {'type': 'integer'},
                                                'description': 'The index of the option in the options array, e.g. [0,1] for the first and second options'
                                            },
                                            'explanation_of_answer': {
                                                'type': 'string',
                                                'description': 'The explanation of why this option is the answer, it should be a key_concept in the md_content'
                                            },
                                        },
                                        'required': ['description_of_problem', 'options', 'answers_options',
                                                     'explanation_of_answer'],
                                        'additionalProperties': False,
                                    },
                                },
                                'required': ['ID', 'content', 'sub_problem_1', 'sub_problem_2'],
                                'additionalProperties': False,
                            }
                        }
                    },
                    'required': ['key_concepts', 'problems'],
                    'additionalProperties': False,
                }
            }
        }
    )
    messages = response.choices[0].message
    data = messages.content
    content_dict = json.loads(data)
    return content_dict


def generate_json_schema_for_no_title(paragraph_count: int, course_name: str, file_name: str):
    if paragraph_count > 5:
        message_content = dedent(
        f""" You are an expert AI assistant specializing in analyzing and structuring educational material. You will be given markdown content from a video in the course "{course_name}", from the file "{file_name}". The text is already divided into paragraphs. Your task is to perform the following actions and format the output as a single JSON object.
        The final JSON object must contain two top-level keys: `sections` and `key_concepts`.
        ### Part 1: Structure the Content
        Analyze the entire text and structure it within the `sections` key.
        1.  **Group into Sections:** Divide the entire text into 3-5 logical sections.
        2.  **Generate Titles:**
            -   For each **section**, create a concise and descriptive title.
            -   For each **original paragraph**, create an engaging title that reflects its main topic.
        3.  **Create a Nested Structure:** The `sections` key must contain an array of section objects.
            -   Each section object must contain a `title` and a nested `paragraphs` array.
            -   Each paragraph object within the nested array must include its `title` and its `original_index` from the source text (starting from 1).
        ### Part 2: Extract Key Concepts and Create Assessments
        Based on the sections you defined in Part 1, distill the core learning points and create a corresponding assessment question for each. This will be structured under the `key_concepts` key.
        1.  **Strict One-to-One Mapping (Most Important):** The relationship between a Source Section and a Key Concept object must be strictly one-to-one. Each Source Section title from Part 1 MUST map to exactly ONE object in the `key_concepts` array.
        2.  **Limited Quantity:** Aggressively merge and consolidate topics. The final output must represent the absolute minimum number of concepts possible, and the count must always be less than 5.
        3.  **Concise Concepts:** The `key_concept` value should be a single, descriptive sentence that captures the main idea of the entire section.
        4.  **Preserve Original Order:** The Key Concept objects must appear in the same order as their corresponding Source Sections appear in Part 1.
        5.  **Generate Follow-up Assessment Question:** For each key concept, you must create an `assessment_question` object. This object is designed to test a student's deep understanding of the concept.
            -   **Challenging Nature:** The question must be difficult. It should require **application, analysis, or synthesis** of the information from the section, not just simple recall.
            -   **Plausible Distractors:** The incorrect options should be plausible and target common student misconceptions related to the topic.
            -   **Structure:** The `assessment_question` object must contain the following four fields:
                -   `question_text`: (String) The full text of the multiple-choice question.
                -   `options`: (Array of Strings) An array containing exactly four possible answers.
                -   `correct_answer`: (String) The exact text of the correct option.
                -   `explanation`: (String) A detailed explanation describing why the correct answer is right and why the other options are incorrect.
        For each concept you extract, provide an object in the `key_concepts` array with the following structure:
        -   `key_concept`: [A single, clear sentence summarizing the core idea of the section.]
        -   `source_section`: [The single, exact title from the section that this concept is derived from.]
        -   `assessment_question`:The structured question object as defined above.""")
        response_format = {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "paragraphs": {
                            "type": "array",
                            "minItems": paragraph_count,
                            "maxItems": paragraph_count,
                            "items": {
                                "type": "object",
                                "properties": {
                                    "title": {"type": "string"},
                                    "paragraph_index": {"type": "integer",
                                                        "description": "The 1-based index from the paragraphs array where this paragraph is located."},
                                },
                                "required": ["title", "paragraph_index"],
                                "additionalProperties": False,
                            },
                        },
                        "sections": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "maxItems": 5,
                                "properties": {
                                    "section_title": {"type": "string"},
                                    "start_paragraph_index": {
                                        "type": "integer",
                                        "description": "The 1-based index from the paragraphs array where this section begins.",
                                        "minimum": 1,
                                        "maximum": paragraph_count,
                                    },
                                },
                                "required": ["section_title", "start_paragraph_index"],
                                "additionalProperties": False,
                            },
                        },
                        "key_concepts": {
                            "type": "array",
                            "description": "A list of key concepts extracted from the sections.",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "concepts": {"type": "string"},
                                    "source_section_title": {"type": "string"},
                                    "assessment_question": {"type": "object",
                                        "properties": {
                                            "question_text": {"type": "string","description": "(String) The full text of the multiple-choice question."},
                                            "options": {"type": "array", "items": {"type": "string"}, "description": "(Array of Strings) An array containing exactly four possible answers."},
                                            "correct_answer": {"type": "array", "items": {"type": "integer"}, "description": "The index of the option in the options array, e.g. [0,1] for the first and second options"},
                                            "explanation": {"type": "string","description": "The explanation of why this option is the answer, it should be a key_concept in the md_content"},
                                        },
                                    },
                                    "content_coverage": {
                                        "type": "array",
                                        'description': 'List only the aspects that the section actually explained with aspect and content.',
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string",
                                                           'minLength': 1,},
                                                "content": {"type": "string"},
                                            },
                                            "required": ["aspect", "content"],
                                            "additionalProperties": False,
                                        },
                                    },
                                },
                                "required": [
                                    "concepts",
                                    "source_section_title",
                                    "assessment_question",
                                    "content_coverage",
                                ],
                                "additionalProperties": False,
                            },
                        },
                    },
                    "required": ["paragraphs", "sections", "key_concepts"],
                    "additionalProperties": False,
                },
            },
        }

    else:
        message_content = dedent(
            f""" You are an expert AI assistant specializing in analyzing and structuring 
            educational material. You will be given markdown content from a video in the course "{course_name}", 
            from the file "{file_name}". The text is already divided into paragraphs. Your task is to perform the 
            following actions and format the output as a single JSON object: 
            ### Part 1: Structure the Content
            **Generate Titles:** - For each pargraph, create only one concise and descriptive title that reflects its main topic. 
            ### Part 2: Extract Key Concepts and Create Assessments
            Based on the paragraphs you defined in Part 1, distill the core learning points and create a corresponding assessment question for each. This will be structured under the `key_concepts` key.
            1.  **Strict One-to-One Mapping (Most Important):** The relationship between a Source Section and a Key Concept object must be strictly one-to-one. Each Source Section title from Part 1 MUST map to exactly ONE object in the `key_concepts` array.
            2.  **Limited Quantity:** Aggressively merge and consolidate topics. The final output must represent the absolute minimum number of concepts possible, and the count must always be less than 5.
            3.  **Concise Concepts:** The `key_concept` value should be a single, descriptive sentence that captures the main idea of the entire section.
            4.  **Preserve Original Order:** The Key Concept objects must appear in the same order as their corresponding Source Sections appear in Part 1.
            5.  **Generate Follow-up Assessment Question:** For each key concept, you must create an `assessment_question` object. This object is designed to test a student's deep understanding of the concept.
                -   **Challenging Nature:** The question must be difficult. It should require **application, analysis, or synthesis** of the information from the section, not just simple recall.
                -   **Plausible Distractors:** The incorrect options should be plausible and target common student misconceptions related to the topic.
                -   **Structure:** The `assessment_question` object must contain the following four fields:
                    -   `question_text`: (String) The full text of the multiple-choice question.
                    -   `options`: (Array of Strings) An array containing exactly four possible answers.
                    -   `correct_answer`: The index of the option in the options array, e.g. [0,1] for the first and second options
                    -   `explanation`: (String) A detailed explanation describing why the correct answer is right and why the other options are incorrect.
            For each concept you extract, provide an object in the `key_concepts` array with the following structure:
            -   `key_concept`: [A single, clear sentence summarizing the core idea of the section.]
            -   `source_section`: [The single, exact title from the section that this concept is derived from.]
            -   `assessment_question`:The structured question object as defined above.""")
        response_format = {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "paragraphs": {
                            "type": "array",
                            "minItems": paragraph_count,
                            "maxItems": paragraph_count,
                            "items": {
                                "type": "object",
                                "properties": {
                                    "title": {"type": "string",
                                              "description": "The title of the paragraph, reflecting its main topic.(each paragraph should have only one title)"},
                                    "paragraph_index": {"type": "integer",
                                                        "description": "The 1-based index from the paragraphs array where this paragraph is located.",
                                                        "maximum": paragraph_count
                                                        },
                                },
                                "required": ["title", "paragraph_index"],
                                "additionalProperties": False,
                            },
                        },
                        "key_concepts": {
                            "type": "array",
                            "description": "A list of key concepts extracted from the sections.",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "concepts": {"type": "string"},
                                    "source_section_title": {"type": "string","description": "Exactly the section title as it appears in the paragraphs, only one title from the list, only start with # can be used, do not include # and any *. Do not treat the line start with * as a title, it is not a title."},
                                    "assessment_question": {
                                    "type": "object",
                                    "properties": {
                                        "question_text": {"type": "string", "description": "(String) The full text of the multiple-choice question."},
                                        "options": {"type": "array", "items": {"type": "string"}, "description": "(Array of Strings) An array containing exactly four possible answers."},
                                        "correct_answer": {"type": "array", "items": {"type": "integer"}, "description": "The index of the option in the options array, e.g. [0,1] for the first and second options"},
                                        "explanation": {"type": "string", "description": "The explanation of why this option is the answer, it should be a key_concept in the md_content"},
                                        },
                                        "required": ["question_text", "options", "correct_answer", "explanation"],
                                        "additionalProperties": False
                                    },
                                    "content_coverage": {
                                        "type": "array",
                                        'description': 'List only the aspects that the section actually explained with aspect and content.',
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string",
                                                            'minLength': 1,
                                                           "description": "The type of information (e.g., Definition, Core Principle, Example, Significance, Mechanism)."},
                                                "content": {"type": "string"},
                                            },
                                            "required": ["aspect", "content"],
                                            "additionalProperties": False,
                                        },
                                    },
                                },
                                "required": [
                                    "concepts",
                                    "source_section_title",
                                    "assessment_question",
                                    "content_coverage",
                                ],
                                "additionalProperties": False,
                            },
                        },
                    },
                    "required": ["paragraphs", "key_concepts"],
                    "additionalProperties": False,
                },
            },
        }
    return message_content, response_format


def get_structured_content_without_title(
        md_content: str, file_name: str, course_name: str,
):
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)

    def paragraph_count(md_text: str) -> int:
        md_text = md_text.strip()
        blocks = re.split(r'\n\s*\n', md_text)
        paragraphs = [b for b in blocks if b.strip()]
        return len(paragraphs)

    paragraph_count = paragraph_count(md_content)
    message_content, response_format = generate_json_schema_for_no_title(paragraph_count, course_name, file_name)
    if not md_content.strip():
        raise ValueError("The content is empty or not properly formatted.")
    response = client.chat.completions.create(
        model="gpt-4.1",
        messages=[{
            "role": "system",
            "content": dedent(message_content)
        },
            {"role": "user", "content": f"{md_content}"},
        ],
        response_format=response_format
    )
    messages = response.choices[0].message
    data = messages.content
    content_dict = json.loads(data)
    return content_dict


def get_structured_content_with_one_title_level(
        md_content: str, file_name: str, course_name: str, index_helper: dict
):
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)
    if not md_content.strip():
        raise ValueError("The content is empty or not properly formatted.")
    md_content = remove_redundant_title(md_content, file_name)
    title_list = [key.replace('"',"'") for d in index_helper for key in d.keys()]
    def paragraph_count(md_text: str) -> int:
        md_text = md_text.strip()
        blocks = re.split(r'\n\s*\n', md_text)
        paragraphs = [b for b in blocks if b.strip()]
        return len(paragraphs)
    if len(title_list) == 1 and paragraph_count(md_content) == 1:
        return {
            "titles_with_levels": [
                {
                    "title": title_list[0],
                    "level_of_title": 1,
                    "paragraph_index": 1
                }
            ],
            "key_concepts": []
        }
    response_format = {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "titles_with_levels": {
                            "type": "array",
                            "description": "A list of titles with their inferred hierarchical level, preserving the original order.",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "title": {
                                        "type": "string",
                                        "enum": title_list,
                                        "description": "MUST be exactly one of the titles from the provided list, without any # symbols and leading/trailing spaces."
                                    },
                                    "level_of_title": {
                                        "type": "integer",
                                        "description": "The inferred hierarchy level (e.g., 1, 2, 3).",
                                    },
                                },
                                "required": ["title", "level_of_title"],
                                "additionalProperties": False,
                            },
                        },
                        "key_concepts": {
                            "type": "array",
                            "description": "A list of key concepts extracted from the sections.",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "concepts": {"type": "string"},
                                    "source_section_title": {
                                        "type": "string",
                                        "enum": title_list,
                                        "description": "*Exactly* the section title as it appears in the markdown (copy‑paste—do **not** alter capitalization, spacing, or punctuation and do not include # and any *)."
                                    },
                                    "assessment_question": {
                                        "type": "object",
                                        "properties": {
                                            "question_text": {"type": "string", "description": "The full text of the multiple-choice question."},
                                            "options": {"type": "array", "items": {"type": "string"}, "description": "An array containing exactly four possible answers."},
                                            "correct_answer": {"type": "array", "items": {"type": "integer"}, "description": "The indices of the correct options in the options array, e.g. [0,1] for the first and second options."},
                                            "explanation": {"type": "string", "description": "A detailed explanation describing why the correct answer is right and why the other options are incorrect."}
                                        },
                                        "required": ["question_text", "options", "correct_answer", "explanation"],
                                        "additionalProperties": False
                                    },
                                    "content_coverage": {
                                        "type": "array",
                                        'description': 'List only the aspects that the section actually explained with aspect and content.',
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string",
                                                            'minLength': 1,
                                                           "description": "The type of information (e.g., Definition, Core Principle, Example, Significance, Mechanism)."},
                                                "content": {"type": "string"},
                                            },
                                            "required": ["aspect", "content"],
                                            "additionalProperties": False,
                                        },
                                    },
                                },
                                "required": [
                                    "concepts",
                                    "source_section_title",
                                    "assessment_question",
                                    "content_coverage",
                                ],
                                "additionalProperties": False,
                            },
                        },
                    },
                    "required": ["titles_with_levels", "key_concepts"],
                    "additionalProperties": False,
                },
            },
        }
    response = client.chat.completions.create(
        model="gpt-4.1",
        messages=[
            {
                "role": "system",
                "content": dedent(
                    f""" You are an expert AI assistant for structuring educational material. You will 
                be given markdown content from the file "{file_name}" for the course "{course_name}". Your task is to 
                analyze this content and produce a structured JSON output. The task has two parts: title structuring 
                and key concept extraction. 
                ### Part 1: Adjust Title Hierarchy Levels
                **Your Goal:** Given the title list, determine the correct semantic hierarchy level for each title based on their logical relationship and content structure.
                **CRITICAL CONSTRAINTS - MUST FOLLOW:**
                - Use EXACTLY the titles from title_list - no additions, no modifications
                - Output title text WITHOUT any '#' symbols 
                - If a title in the list has '#' symbols, remove them completely
                - Do NOT create any new titles that aren't in the provided list
                - Do NOT include any text that doesn't appear in {title_list}
                **FORBIDDEN - Do NOT do these:**
                ❌ Adding titles not in the list
                ❌ Including # symbols
                ❌ Modifying title text in any way
                **Maintain Logical Flow:** Each title's level should make sense relative to the titles before and after it
                ### Part 2: Extract High-Level Key Concepts for Overview
                Your goal is to create a high-level summary of the entire document by identifying a small, curated set of its most important concepts. This should serve as a "big picture" overview for a student.
                CRITICAL CONSTRAINTS - YOU MUST FOLLOW:
                1.  **Strict One-to-One Mapping (Most Important):** The relationship between a Source Section and a Key Concept must be strictly one-to-one. Each chosen Source Section title MUST map to exactly ONE Key Concept. It is forbidden to generate multiple Key Concepts from the same single Source Section.
                2.  **Limited Quantity:** Your most important task is to aggressively merge and consolidate topics. Actively seek to unify related ideas under a single, overarching key concept. Before creating a new concept, you must first determine if its core idea can be logically absorbed by another. The final output must represent the absolute minimum number of concepts possible, and the count must always be less than 5.
                3.  **No Hierarchical Overlap:** If you choose a main section title (e.g., "Chapter 1"), you cannot also choose one of its sub-sections (e.g., "Section 1.1").
                4.  **Concise Concepts:** The key concept should be a single, descriptive sentence that captures the main idea of the entire section block.
                5.  **Preserve Original Order** Key Concepts must appear in the same order as their corresponding Source Sections appear in the original markdown.Do not reorder, merge across, or reshuffle the sequence.
                6.  **Generate Follow-up Assessment Question:** For each key concept, you must create an `assessment_question` object. This object is designed to test a student's deep understanding of the concept.
                    -   **Challenging Nature:** The question must be difficult. It should require **application, analysis, or synthesis** of the information from the section, not just simple recall.
                    -   **Plausible Distractors:** The incorrect options should be plausible and target common student misconceptions related to the topic.
                    -   **Structure:** The `assessment_question` object must contain the following four fields:
                        -   `question_text`: (String) The full text of the multiple-choice question.
                        -   `options`: (Array of Strings) An array containing exactly four possible answers.
                        -   `correct_answer`: (Array of Integers) The indices of the correct options in the options array, e.g. [0,1] for the first and second options.
                        -   `explanation`: (String) A detailed explanation describing why the correct answer is right and why the other options are incorrect.
                For each concept you extract, provide ONLY the following information:
                Key Concept: [A single, clear sentence summarizing the core idea of the section.]
                Source Section: [The single, exact title from {title_list} that this concept is derived from.]
                Assessment Question: The structured question object as defined above."""),
            },
            {"role": "user", "content": f"{md_content} "},
        ],
        response_format=response_format,
    )
    messages = response.choices[0].message
    data = messages.content
    content_dict = json.loads(data)
    return content_dict


def get_title_list(md_content: str):
    lines = md_content.split("\n")
    titles = []
    for line in lines:
        if line.startswith("#"):
            title = line[1:]
            if line.startswith('*'):
                title = line.lstrip('*').rstrip('*').strip()
            titles.append(title)
    return titles


def remove_redundant_title(md_content: str, file_name: str):
    normalized_filename = file_name.replace("-", " ").replace("_", " ")
    first_line = md_content.split("\n")[0]
    if first_line.startswith("# "):
        title_text = first_line.lstrip("# ").strip()
        if normalized_filename == title_text.lower():
            print(f"Found and removing redundant title in '{file_name}'.")
            remaining_lines = md_content.split("\n")[1:]
            while remaining_lines and remaining_lines[0].strip() == "":
                remaining_lines.pop(0)
            heading_levels_found = set()
            for line in remaining_lines:
                stripped_line = line.lstrip()
                if stripped_line.startswith("#"):
                    level = 0
                    while level < len(stripped_line) and stripped_line[level] == "#":
                        level += 1
                    heading_levels_found.add(level)

            final_lines = []
            if len(heading_levels_found) > 1:
                print(
                    "Multiple heading levels found. Promoting subsequent headings by one level."
                )
                for line in remaining_lines:
                    stripped_line = line.lstrip()
                    if stripped_line.startswith("##"):
                        first_hash_index = line.find("#")
                        new_line = (
                                line[:first_hash_index] + line[first_hash_index + 1:]
                        )
                        final_lines.append(new_line)
                    else:
                        final_lines.append(line)
            else:
                print("No promotion needed (headings are uniform or absent).")
                final_lines = remaining_lines
            return final_lines

    print(f"No redundant title found in '{file_name}'.")
    final_lines = md_content
    return final_lines

def apply_structure_for_no_title(md_content: str, content_dict):
    original_paragraphs = [p.strip() for p in md_content.split("\n\n") if p.strip()]
    md_parts = []
    content_dict['titles_with_levels'] = []

    if 'sections' in content_dict:
        section_starts = {
            s["start_paragraph_index"]: s["section_title"]
            for s in content_dict.get("sections")
        }
    else:
        section_starts = {}

    for paragraph in sorted(
            content_dict["paragraphs"], key=lambda p: p["paragraph_index"]
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
        # Check if section_starts not empty to determine heading level
        if section_starts:
            level = 2

        md_parts.append(f"{'#' * level} {title}\n\n")
        content_dict['titles_with_levels'].append({
            "title": title,
            "level_of_title": level,
            "paragraph_index": p_index,
        })

        # Find the correct paragraph content by matching title
        content_index =p_index - 1  # Convert to 0-based index

        md_parts.append(f"{original_paragraphs[content_index]}\n\n")

    output_content = "".join(md_parts)
    return output_content

def fix_title_levels(mapping_list):
    """
    Fix title levels in the mapping list to ensure they are sequential.
    """
    last_level = 0
    for i in range(len(mapping_list) - 1):
        current_level = mapping_list[i]["level_of_title"]
        mapping_list[i]['title']=mapping_list[i]['title'].strip()
        if current_level > last_level + 1:
            diff = current_level - (last_level + 1)
            j = i
            while j < len(mapping_list) and mapping_list[j]["level_of_title"] >= current_level:
                mapping_list[j]["level_of_title"] -= diff
                j += 1
        last_level = mapping_list[i]["level_of_title"]
    return mapping_list


def apply_structure_for_one_title(md_content: str, content_dict):
    mapping_list = content_dict.get("titles_with_levels")
    mapping_list = fix_title_levels(mapping_list)
    i = 0
    lines = md_content.split("\n")
    title_pattern = re.compile(r"^(?P<hashes>#{1,6})\s+(?P<title>\S.*?)$")
    new_lines = []
    for line in lines:
        match = title_pattern.match(line.strip())
        if match:
            raw_title = match.group("title").strip()
            if raw_title.replace('"',"'") not in [d['title'] for d in mapping_list] and raw_title not in [d['title'] for d in mapping_list]:
                logger.warning(f"Title '{raw_title}' not found in mapping list, skipping. Maybe it is not a redundant title.")
                continue
            new_level = mapping_list[i]["level_of_title"]
            new_lines.append(f"{'#' * new_level} {raw_title}")
            i += 1
        else:
            new_lines.append(line)
        md_content = "\n".join(new_lines)
    return md_content


def save_key_concept_to_metadata(json_dict, metadata_path: Path):
    if metadata_path.exists():
        with open(metadata_path, "r") as metadata_file:
            data = yaml.safe_load(metadata_file)
            if not data:
                logger.warning("No URL found in metadata file")
                data = {}
    else:
        logger.warning("No metadata file exists, creating a new one.")
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        data = {}
    print("Creating a new metadata file at:", metadata_path)
    key_concept = json_dict["key_concepts"]
    key_concept[0]["source_section_title"] = key_concept[0][
        "source_section_title"
    ].strip()
    data["key_concept"] = key_concept
    with open(metadata_path, "w") as metadata_file:
        yaml.safe_dump(data, metadata_file, default_flow_style=False)


def get_only_key_concepts(md_content: str, index_helper: dict):
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)
    title_list = [key.replace('"',"'") for d in index_helper for key in d.keys()] if index_helper else get_title_list(md_content)
    response_format = {
        "type": "json_schema",
        "json_schema": {
            "name": "course_content_knowledge_sorting",
            "strict": True,
            "schema": {
                "type": "object",
                "properties": {
                    "key_concepts": {
                        "type": "array",
                        'description': 'A list of key concepts extracted from the content.',
                        "items": {
                            "type": "object",
                            "properties": {
                                "concepts": {"type": "string",
                                             "description": "A single, clear phrase summarizing the core idea of the section."},
                                "source_section_title": {"type": "string",
                                                         "enum": title_list,
                                                         "description": "MUST be exactly one of the titles from the provided list."},
                                "assessment_question": {
                                    "type": "object",
                                    "properties": {
                                        "question_text": {"type": "string", "description": "The full text of the multiple-choice question."},
                                        "options": {"type": "array", "items": {"type": "string"}, "description": "An array containing exactly four possible answers."},
                                        "correct_answer": {"type": "array", "items": {"type": "integer"}, "description": "The indices of the correct options in the options array, e.g. [0,1] for the first and second options."},
                                        "explanation": {"type": "string", "description": "A detailed explanation describing why the correct answer is right and why the other options are incorrect."}
                                    },
                                    "required": ["question_text", "options", "correct_answer", "explanation"],
                                    "additionalProperties": False
                                },
                                "content_coverage": {
                                    "type": "array",
                                    "description": "A list of key aspects explained within that section block.",
                                    "items": {
                                        "type": "object",
                                        "properties": {
                                            "aspect": {"type": "string",
                                                        "minLength": 1,
                                                       "description": "The type of information (e.g., Definition, Core Principle, Example, Significance, Mechanism)."},
                                            "content": {"type": "string"},
                                        },
                                        "required": ["aspect", "content"],
                                        "additionalProperties": False,
                                    },
                                },
                            },
                            "required": [
                                "concepts",
                                "source_section_title",
                                "assessment_question",
                                "content_coverage",
                            ],
                            "additionalProperties": False,
                        },
                    }
                },
                "required": ["key_concepts"],
                "additionalProperties": False,
            },
        },
    }

    response = client.chat.completions.create(
        model="gpt-4.1",
        messages=[
            {
                "role": "system",
                "content": dedent(
                f"""Extract High-Level Key Concepts for Overview
            Your goal is to create a high-level summary of the entire document by identifying a small, curated set of its most important concepts. This should serve as a "big picture" overview for a student.
            CRITICAL CONSTRAINTS - YOU MUST FOLLOW:
            1.  **Strict One-to-One Mapping (Most Important):** The relationship between a Source Section and a Key Concept must be strictly one-to-one. Each chosen Source Section title MUST map to exactly ONE Key Concept. It is forbidden to generate multiple Key Concepts from the same single Source Section.
            2.  **Limited Quantity:** Your most important task is to aggressively merge and consolidate topics. Actively seek to unify related ideas under a single, overarching key concept. Before creating a new concept, you must first determine if its core idea can be logically absorbed by another. The final output must represent the absolute minimum number of concepts possible, and the count must always be less than 5.
            3.  **No Hierarchical Overlap:** If you choose a main section title (e.g., "Chapter 1"), you cannot also choose one of its sub-sections (e.g., "Section 1.1").
            4.  **Concise Concepts:** The key concept should be a single, descriptive sentence that captures the main idea of the entire section block.                
            5.  **Preserve Original Order** Key Concepts must appear in the same order as their corresponding Source Sections appear in the original markdown.Do not reorder, merge across, or reshuffle the sequence.
            6.  **Generate Follow-up Assessment Question:** For each key concept, you must create an `assessment_question` object. This object is designed to test a student's deep understanding of the concept.
                -   **Challenging Nature:** The question must be difficult. It should require **application, analysis, or synthesis** of the information from the section, not just simple recall.
                -   **Plausible Distractors:** The incorrect options should be plausible and target common student misconceptions related to the topic.
                -   **Structure:** The `assessment_question` object must contain the following four fields:
                    -   `question_text`: (String) The full text of the multiple-choice question.
                    -   `options`: (Array of Strings) An array containing exactly four possible answers.
                    -   `correct_answer`: (Array of Integers) The indices of the correct options in the options array, e.g. [0,1] for the first and second options.
                    -   `explanation`: (String) A detailed explanation describing why the correct answer is right and why the other options are incorrect.
            For each concept you extract, provide ONLY the following information:
            Key Concept: [A single, clear sentence summarizing the core idea of the section.]
            Source Section: [The single, exact title from {title_list} that this concept is derived from.]
            Assessment Question: The structured question object as defined above."""),
            },
            {"role": "user", "content": f"{md_content}"},
        ],
        response_format=response_format
    )
    messages = response.choices[0].message
    data = messages.content
    content_dict = json.loads(data)
    return content_dict


def add_titles_to_json(index_helper, json_file_path):
    """
    Insert title entries into a transcript list based on timing information.

    Args:
        json_file_path: Path to the JSON file containing the transcript data.
        index_helper: Dictionary with title as key and start time as value

    Returns:
        New list with title entries inserted at appropriate positions
    """

    with open(json_file_path, "r") as json_file:
        transcript_list = json.load(json_file)

    # Convert index_helper to a list of tuples for easier processing
    # Each tuple contains (title, start_time)
    titles_to_insert = [(title, start_time[0]) for title, start_time in index_helper.items()]
    titles_to_insert.sort(key=lambda x: float(x[1]))

    current_index = 0
    for title, start_time in titles_to_insert:
        while current_index < len(transcript_list) and float(transcript_list[current_index]["start time"]) < float(start_time):
            current_index += 1

        # Create the title entry
        title_entry = {
            "start time": get_previous_end_time(transcript_list, current_index),
            "end time": get_next_start_time(transcript_list, current_index),
            "speaker": f"title-{len(title)}",
            "text content": f"{title[-1]}",
        }

        # Insert the title entry
        transcript_list.insert(current_index, title_entry)
        # Update the current index to the next position
        current_index += 1

    with open(json_file_path, "w") as json_file:
        json.dump(transcript_list, json_file, indent=4)


def find_insertion_position(transcript_list, target_time):
    """Find where to insert a title based on its start time."""
    target_seconds = float(target_time)

    for i, entry in enumerate(transcript_list):
        entry_start_seconds = float(entry["start time"])
        if entry_start_seconds >= target_seconds:
            return i

    # If no position found, insert at the end
    return len(transcript_list)


def get_previous_end_time(transcript_list, position):
    """Get the end time of the previous entry, or '0.000' if at the beginning."""
    if position == 0:
        return 0.0
    return transcript_list[position - 1]["end time"]

def get_next_start_time(transcript_list, position):
    """Get the start time of the next entry, or '0.000' if at the end."""
    if position >= len(transcript_list) - 1:
        return get_previous_end_time(transcript_list, position)
    return transcript_list[position]["start time"]