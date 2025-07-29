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
        md_content: str, file_name: str, course_name: str,
):
    # TODO : add all titles form index helper to the message content

    get_title_list(md_content)
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)
    if not md_content.strip():
        raise ValueError("The content is empty or not properly formatted.")
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
                    Your goal is to identify and explain the key concepts in each section to help a student recap the material. 
                    For each Key Concept, provide the following information: 
                    - **Key Concept:** A descriptive phrase or sentence that clearly captures the main idea 
                    - **Source Section:** The specific section title in the material where this concept is discussed.(Should be only one section title only lines started with # can be titles) 
                    - **Content Coverage:** List only the aspects that the section actually explained with aspect and content. 
                    - Some good examples of aspect: Definition, How it works, What happened, Why is it important, etc 
                    - The content should be directly from the section.

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
            {"role": "user", "content": f"{md_content}\n title_list: {get_title_list(md_content)} "},
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
                                                             "description": "*Exactly* the section title as it appears in the markdown (copy‑paste—do **not** alter capitalization, spacing, or punctuation and do not include # and any *)."},
                                    "content_coverage": {
                                        "type": "array",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string"},
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
    if paragraph_count >= 5:
        message_content = dedent(
            f""" You are an expert AI assistant specializing in analyzing and structuring 
        educational material. You will be given markdown content from a video in the course "{course_name}", 
        from the file "{file_name}". The text is already divided into paragraphs. Your task is to perform the 
        following actions and format the output as a single JSON object: 1.  **Group into Sections:** Analyze 
        the entire text and divide it into **at most 5 logical sections**. 2.  **Generate Titles:** - For 
        each **section**, create a concise and descriptive title. - For each **original paragraph**, 
        create an engaging title that reflects its main topic. 3.  **Create a Nested Structure:** The JSON 
        output must have a `sections` array. - Each element in the `sections` array is an object representing 
        one section. - **Crucially, each section object must contain its own `paragraphs` array.** This 
        nested array should list all the paragraphs that belong to that section. - Each paragraph object 
        within the nested array must include its title and its **original index** from the source text (
        starting from 1). ### Part 2: Extract Key Concepts 
        Your goal is to identify and explain the key concepts in each section to help a student recap the material. 
        For each Key Concept, provide the following information: 
        - **Key Concept:** A descriptive phrase or sentence that clearly captures the main idea 
        - **Source Section:** The specific section title in the material where this concept is discussed.(Should be only one section title only lines started with # can be titles) 
        - **Content Coverage:** List only the aspects that the section actually explained with aspect and content. 
        - Some good examples of aspect: Definition, How it works, What happened, Why is it important, etc 
        - The content should be directly from the section. """
        )
        json_schema = {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "paragraphs": {
                            "type": "array",

                            "items": {
                                "type": "object",
                                "properties": {
                                    "title": {"type": "string"},
                                    "paragraph_index": {"type": "integer"},
                                },
                                "required": ["title", "paragraph_index"],
                                "additionalProperties": False,
                            },
                        },
                        "sections": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "maxItem": 5,
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
                            "items": {
                                "type": "object",
                                "properties": {
                                    "concepts": {"type": "string"},
                                    "source_section_title": {"type": "string"},
                                    "content_coverage": {
                                        "type": "array",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string"},
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
            following actions and format the output as a single JSON object:  1.  **Generate Titles:** - For 
            each pargraph, create a concise and descriptive title that reflects its main topic. 
            ### Part 2: Extract Key Concepts 
            Your goal is to identify and explain the key concepts in each section to help a student recap the material. 
            For each Key Concept, provide the following information: 
            - **Key Concept:** A descriptive phrase or sentence that clearly captures the main idea 
            - **Source Section:** The specific section title in the material where this concept is discussed.(Should be only one section title only lines started with # can be titles) 
            - **Content Coverage:** List only the aspects that the section actually explained with aspect and content. 
            - Some good examples of aspect: Definition, How it works, What happened, Why is it important, etc 
            - The content should be directly from the section. """
        )
        json_schema = {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "paragraphs": {
                            "type": "array",
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
                        "key_concepts": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "concepts": {"type": "string"},
                                    "source_section_title": {"type": "string"},
                                    "content_coverage": {
                                        "type": "array",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string"},
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
    return message_content, json_schema


def get_structured_content_without_title(
        md_content: str, file_name: str, course_name: str,
):
    # TODO : add a check for paragraphs count if less than 5 then do not generate section
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)

    def paragraph_count(md_text: str) -> int:
        md_text = md_text.strip()
        blocks = re.split(r'\n\s*\n', md_text)
        paragraphs = [b for b in blocks if b.strip()]
        return len(paragraphs)

    paragraph_count = paragraph_count(md_content)
    message_content, json_schema = generate_json_schema_for_no_title(paragraph_count, course_name, file_name)
    if not md_content.strip():
        raise ValueError("The content is empty or not properly formatted.")
    response = client.chat.completions.create(
        model="gpt-4.1",
        messages=[{
            "role": "system",
            "content": message_content
        },
            {"role": "user", "content": f"{md_content}"},
        ],
        response_format=json_schema
    )
    messages = response.choices[0].message
    data = messages.content
    content_dict = json.loads(data)
    return content_dict


def get_structured_content_with_one_title_level(
        md_content: str, file_name: str, course_name: str
):
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)
    if not md_content.strip():
        raise ValueError("The content is empty or not properly formatted.")
    md_content = remove_redundant_title(md_content, file_name)
    title_list = get_title_list(md_content)

    def generate_json_schema(title_list):
        return {
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": False,
                "schema": {
                    "type": "object",
                    "properties": {
                        "titles_with_levels": {
                            "type": "array",
                            "description": "A list of titles with their inferred hierarchical level, preserving the original order.",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "title": {"type": "string", "enum": title_list},
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
                            "items": {
                                "type": "object",
                                "properties": {
                                    "concepts": {"type": "string"},
                                    "source_section_title": {"type": "string"},
                                    "content_coverage": {
                                        "type": "array",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string"},
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
                and key concept extraction. ### Part 1: Correct Title Hierarchy **Your Goal:** The markdown's title 
                hierarchy is likely flat (e.g., every title starts with a single '#'). Your job is to determine the 
                correct semantic level for each of these titles. **Crucial Rule:** - A line is considered a title if, 
                and only if, it begins with one or more '#' characters in the provided text. Do NOT invent new titles 
                or treat any other text as a title. **How to Determine the Correct Level (1, 2, 3, etc.):** 1.  
                **Analyze Logical Structure:** Read the titles in sequence to understand the flow of the document. A 
                title that introduces a new, major section is a high level (e.g., level 1). A title that discusses a 
                sub-point of the previous title is a lower level (e.g., level 2 or 3). 2.  **Preserve Order:** The 
                titles in your JSON output must be in the exact same order they appear in the source text. 3.  
                **Output Format:** In the JSON, provide the clean title text (without the '#') and the integer level 
                you have assigned.

                ### Part 2: Extract Key Concepts 
                Your goal is to identify and explain the key concepts in each section to help a student recap the material. 
                For each Key Concept, provide the following information: 
                - **Key Concept:** A descriptive phrase or sentence that clearly captures the main idea 
                - **Source Section:** The specific section title in the material where this concept is discussed.(Should be only one section title only lines started with # can be titles) 
                - **Content Coverage:** List only the aspects that the section actually explained with aspect and content. 
                - Some good examples of aspect: Definition, How it works, What happened, Why is it important, etc 
                - The content should be directly from the section. """
                ),
            },
            {"role": "user", "content": f"{md_content}\n title_list: {title_list} "},
        ],
        response_format=generate_json_schema(title_list),
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
        if p_index in section_starts:
            section_title = section_starts[p_index].strip()
            md_parts.append(f"# {section_title}\n\n")
            content_dict['titles_with_levels'].append({
                "title": section_title,
                "level_of_title": 1,
                "paragraph_index": p_index,
            })
            level_of_para_title = 2
        else:
            level_of_para_title = 1
        md_parts.append(f"{'#' * level_of_para_title} {p_title}\n\n")
        content_dict['titles_with_levels'].append({
            "title": p_title,
            "level_of_title": level_of_para_title,
            "paragraph_index": p_index,
        })
        content_index = p_index - 1
        if 0 <= content_index < len(original_paragraphs):
            md_parts.append(f"{original_paragraphs[content_index]}\n\n")
        else:
            md_parts.append(f"[Content for paragraph {p_index} not found]\n\n")

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
    title_pattern = re.compile(r"^(?P<hashes>#+)\s*(?P<title>.+?)\s*$")
    new_lines = []
    for line in lines:
        match = title_pattern.match(line.strip())
        if match:
            raw_title = match.group("title").strip()
            assert raw_title == mapping_list[i][
                "title"].strip(), f"Title mismatch: {raw_title} != {mapping_list[i]['title']}"
            new_level = mapping_list[i]["level_of_title"]
            new_lines.append(f"{'#' * new_level} {raw_title}")
            i += 1
        else:
            new_lines.append(line)
        md_content = "\n\n".join(new_lines)

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


def get_only_key_concepts(md_content: str, file_name: str, course_name: str):
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)
    response = client.chat.completions.create(
        model="gpt-4.1",
        messages=[
            {
                "role": "system",
                "content": dedent(
                    f"""
                    You are an expert AI assistant specializing in analyzing educational material. You will be given markdown content from a course "{course_name}", from the file "{file_name}".
                    Your task is to perform the following actions and format the output as a single JSON object:
                    Your goal is to identifying and explaining the key concepts for each section title to help a student recap the material.
                    For each Key Concept, provide the following information:
                   - **Key Concept:** A descriptive phrase or sentence that clearly captures the main idea
                   - **Source Section:** The exact one section title in the material which this concept is about.(Only line started with # can be titles)
                   - **Content Coverage:** List only the aspects that the section actually explained with aspect and content. 
                     - Some good examples of aspect: Definition, How it works, What happened, Why is it important, etc
                     - The content also should be from the section.
                    """
                ),
            },
            {"role": "user", "content": f"{md_content}, title_list: {get_title_list(md_content)}"},
        ],
        response_format={
            "type": "json_schema",
            "json_schema": {
                "name": "course_content_knowledge_sorting",
                "strict": True,
                "schema": {
                    "type": "object",
                    "properties": {
                        "key_concepts": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "concepts": {"type": "string"},
                                    "source_section_title": {"type": "string"},
                                    "content_coverage": {
                                        "type": "array",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "aspect": {"type": "string"},
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
        },
    )
    messages = response.choices[0].message
    data = messages.content
    content_dict = json.loads(data)
    return content_dict