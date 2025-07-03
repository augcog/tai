import json
from textwrap import dedent
from openai import OpenAI
from openai.types.chat import ChatCompletion, ChatCompletionMessage
from typing import Union
from pathlib import Path
import re
import os


class OpenAIChatMarkdownStructure:
    def __init__(
        self, api_key: str, file_path: Union[str, Path], file_type: str, course_name
    ):
        self.client = OpenAI(api_key=api_key)
        self.file_path = file_path
        self.file_type = file_type
        self.course_name = course_name

    def remove_redundant_title(self, file_path: Union[str, Path]) -> bool | None:
        file_path = Path(file_path)
        normalized_filename = file_path.stem.lower().replace("-", " ").replace("_", " ")
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                lines = f.readlines()
        except FileNotFoundError:
            print(f"Error: File not found at '{file_path}'")
            return None
        if not lines:
            return False
        first_line = lines[0]
        if first_line.startswith("# "):
            title_text = first_line.lstrip("# ").strip()
            normalized_title = title_text.lower()
            if normalized_filename == normalized_title:
                print(f"Found redundant title in '{file_path}'. Removing and saving...")
                remaining_lines = lines[1:]
                while remaining_lines and remaining_lines[0].strip() == "":
                    remaining_lines.pop(0)
                new_content = "".join(remaining_lines)
                try:
                    with open(file_path, "w", encoding="utf-8") as f:
                        f.write(new_content)
                    print(f"Successfully updated '{file_path}'.")
                    return True
                except IOError as e:
                    print(f"Error writing back to file '{file_path}': {e}")
                    return None
        print(f"No redundant title found in '{file_path}'. No changes made.")
        return False

    def _markdown_structure(self):
        if self.file_type == "pdf":
            self.remove_redundant_title(self.file_path)
            response = self.pdf_markdown_structure(
                course_name=self.course_name, file_path=self.file_path
            )
            json_file_path = self.file_path.with_suffix(".json")
            self.save_chat_response_to_file(response, json_file_path)
            output_file_path = (
                self.file_path.parent
                / f"{self.file_path.stem}_structured{self.file_path.suffix}"
            )
            self.insert_title_levels(
                md_path=self.file_path,
                mapping_path=json_file_path,
                output_path=output_file_path,
            )
        if self.file_type == "mp4" or self.file_type == "mp3":
            response = self.video_markdown_structure(
                course_name=self.course_name, file_path=self.file_path
            )
            json_file_path = Path(self.file_path).with_suffix(".json")
            self.save_chat_response_to_file(response, json_file_path)
            output_file_path = (
                self.file_path.parent
                / f"{self.file_path.stem}_structured{self.file_path.suffix}"
            )
            self.insert_sections_and_paragraph_titles(
                original_md_path=self.file_path,
                json_path=json_file_path,
                output_md_path=output_file_path,
            )
        else:
            output_file_path = self.file_path
        return output_file_path

    def pdf_markdown_structure(
        self, course_name: str, file_path: str
    ) -> ChatCompletionMessage:
        with open(file_path, "r", encoding="utf-8") as file:
            content = file.read()
        if not content.strip():
            raise ValueError("The content is empty or not properly formatted.")
        file_name = str(file_path).split("/")[-1]
        response = self.client.chat.completions.create(
            model="gpt-4.1",
            messages=[
                {
                    "role": "system",
                    "content": dedent(
                        f"""
            You are an expert AI assistant for structuring educational material. You will be given markdown content from the file "{file_name}" for the course "{course_name}".
            Your task is to analyze this content and produce a structured JSON output. The task has two parts: title structuring and key concept extraction.
            ### Part 1: Correct Title Hierarchy
            **Your Goal:** The markdown's title hierarchy is likely flat (e.g., every title starts with a single '#'). Your job is to determine the correct semantic level for each of these titles.
            **Crucial Rule:**
            - A line is considered a title if, and only if, it begins with one or more '#' characters in the provided text. Do NOT invent new titles or treat any other text as a title.
            **How to Determine the Correct Level (1, 2, 3, etc.):**
            1.  **Analyze Logical Structure:** Read the titles in sequence to understand the flow of the document. A title that introduces a new, major section is a high level (e.g., level 1). A title that discusses a sub-point of the previous title is a lower level (e.g., level 2 or 3).
            2.  **Use Numbering as a Strong Cue:** Look for explicit numbering like "Chapter 1", "2.1 Section", "3.1.2 Detail". This numbering directly indicates the intended hierarchy.
            3.  **Recognize Standard Sections:** Titles like "Introduction", "Conclusion", "Summary", and "References" are almost always level 1.
            4.  **Preserve Order:** The titles in your JSON output must be in the exact same order they appear in the source text.
            5.  **Output Format:** In the JSON, provide the clean title text (without the '#') and the integer level you have assigned.

            ### Part 2: Extract Key Concepts
            Your goal is to help a student truly understand. You will provide a list of `explanation_points`. For each concept, **you decide which types of explanations are most effective.****Possible `type` values for an explanation point:**
            -   **Definition:** A clear, formal or informal definition.
            -   **Analogy:** A relatable comparison to help build intuition (e.g., "Think of it like...").
            -   **How it Works:** A step-by-step breakdown of a process.
            -   **Why it's Important:** The significance or "so what?" factor for a student.
            -   **Code Example:** A short, clear code snippet (for technical topics).
            -   **Real-world Example:** A concrete example of the concept in action.
            -   **Common Pitfall:** A warning about common mistakes or misunderstandings.
        """
                    ),
                },
                {"role": "user", "content": f"{content}"},
            ],
            response_format={
                "type": "json_schema",
                "json_schema": {
                    "name": "course_content_knowledge_sorting",
                    "strict": True,
                    "description": "Structures course content into titles and knowledge terms.",
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
                                            "description": "The clean text of the title, without any '#' characters.",
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
                            "key_concept": {
                                "type": "array",
                                "description": "A glossary of key terms with detailed explanations.",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "term": {"type": "string"},
                                        "explanation_points": {
                                            "type": "string",
                                        },
                                    },
                                    "required": ["term", "explanation_points"],
                                    "additionalProperties": False,
                                },
                            },
                        },
                        "required": ["titles_with_levels", "key_concept"],
                        "additionalProperties": False,
                    },
                },
            },
        )
        print(response.choices[0].message)
        return response.choices[0].message

    def insert_title_levels(
        self,
        md_path: Union[Path, str],
        mapping_path: Union[Path, str],
        output_path: Union[Path, str],
    ):
        raw = json.load(open(mapping_path, "r", encoding="utf-8"))
        if "titles_with_levels" not in raw and "content" in raw:
            raw = json.loads(raw["content"])
        mapping_list = raw.get("titles_with_levels")
        if not mapping_list:
            raise KeyError("Can not find the titles with levels")

        title_level_map = {}
        for item in mapping_list:
            if "titles" in item and "levels_of_titles" in item:
                t, lvl = item["titles"].strip(), int(item["levels_of_titles"])
            elif "title" in item and "level_of_title" in item:
                t, lvl = item["title"].strip(), int(item["level_of_title"])
            else:
                raise KeyError(f"Can not found {item}")
            title_level_map[t] = lvl

        lines = Path(md_path).read_text(encoding="utf-8").splitlines(keepends=True)

        title_pattern = re.compile(r"^(?P<hashes>#+)\s*(?P<title>.+?)\s*$")

        new_lines = []
        for line in lines:
            m = title_pattern.match(line)
            if m:
                raw_title = m.group("title")
                if raw_title in title_level_map:
                    new_lvl = title_level_map[raw_title]
                    new_lines.append(f"{'#' * new_lvl} {raw_title}\n")
                    continue
            new_lines.append(line)

        Path(output_path).write_text("".join(new_lines), encoding="utf-8")
        print(f"✅ writen ：{output_path}")

    def save_chat_response_to_file(
        self,
        response: Union[ChatCompletion, ChatCompletionMessage, dict, str],
        JSON_file_path: Union[str, Path],
        *,
        full: bool = True,
    ) -> dict:
        data = None
        if isinstance(response, dict):
            data = response
        elif isinstance(response, str):
            try:
                data = json.loads(response)
            except json.JSONDecodeError:
                data = {"content": response}

        else:
            if full and hasattr(response, "to_dict"):
                data = response.to_dict()
            elif full and hasattr(response, "dict"):
                data = response.dict()
            elif not full and hasattr(response, "choices"):
                msg = response.choices[0].message
                if hasattr(msg, "to_dict"):
                    data = msg.to_dict()
                elif hasattr(msg, "dict"):
                    data = msg.dict()
                else:
                    data = {
                        "role": getattr(msg, "role", None),
                        "content": getattr(msg, "content", None),
                    }
            elif isinstance(response, ChatCompletionMessage):
                if hasattr(response, "to_dict"):
                    data = response.to_dict()
                elif hasattr(response, "dict"):
                    data = response.dict()
                else:
                    data = {"role": response.role, "content": response.content}
        if not isinstance(data, dict):
            raise ValueError(
                "Could not normalize response to a dict. "
                "Supported types: dict, JSON string, ChatCompletion, ChatCompletionMessage."
            )
        with open(JSON_file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        print(f"Wrote JSON to {JSON_file_path}")
        return data

    def video_markdown_structure(
        self, course_name: str, file_path: str
    ) -> ChatCompletionMessage:
        with open(file_path, "r", encoding="utf-8") as file:
            content = file.read()
        if not content.strip():
            raise ValueError("The content is empty or not properly formatted.")
        file_name = str(file_path).split("/")[-1]

        response = self.client.chat.completions.create(
            model="gpt-4.1",
            messages=[
                {
                    "role": "system",
                    "content": dedent(
                        f"""
                                You are an expert AI assistant specializing in analyzing and structuring educational material. You will be given markdown content from a video in the course "{course_name}", from the file "{file_name}". The text is already divided into paragraphs.
                                Your task is to perform the following actions and format the output as a single JSON object:
                                1.  **Group into Sections:** Analyze the entire text and divide it into **3 to 5 logical sections**.
                                2.  **Generate Titles:**
                                    - For each **section**, create a concise and descriptive title.
                                    - For each **original paragraph**, create an engaging title that reflects its main topic.
                                3.  **Create a Nested Structure:** The JSON output must have a `sections` array.
                                    - Each element in the `sections` array is an object representing one section.
                                    - **Crucially, each section object must contain its own `paragraphs` array.** This nested array should list all the paragraphs that belong to that section.
                                    - Each paragraph object within the nested array must include its title and its **original index** from the source text (starting from 1).
                                 ### Part 2: Extract Key Concepts
                                Your goal is to help a student truly understand. You will provide a list of `explanation_points`. For each section, **you decide which types of explanations are most effective.****Possible `type` values for an explanation point:**
                                -   **Definition:** A clear, formal or informal definition.
                                -   **Analogy:** A relatable comparison to help build intuition (e.g., "Think of it like...").
                                -   **How it Works:** A step-by-step breakdown of a process.
                                -   **Why it's Important:** The significance or "so what?" factor for a student.
                                -   **Code Example:** A short, clear code snippet (for technical topics).
                                -   **Real-world Example:** A concrete example of the concept in action.
                                -   **Common Pitfall:** A warning about common mistakes or misunderstandings.
                                """
                    ),
                },
                {"role": "user", "content": f"{content} "},
            ],
            response_format={
                "type": "json_schema",
                "json_schema": {
                    "name": "course_content_knowledge_sorting",
                    "strict": True,
                    "description": "A nested structure of course content with sections, paragraphs, and key knowledge terms.",
                    "schema": {
                        "type": "object",
                        "properties": {
                            "sections": {
                                "type": "array",
                                "description": "An array of content sections, each containing its own paragraphs.",
                                "minItems": 3,
                                "maxItems": 5,
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "section_index": {
                                            "type": "integer",
                                            "description": "The sequential index of the section (e.g., 1, 2, 3).",
                                            "minimum": 1,
                                        },
                                        "section_title": {
                                            "type": "string",
                                            "description": "A concise title for the entire section.",
                                        },
                                        "title_level": {
                                            "type": "integer",
                                            "description": "The markdown heading level for the section title.",
                                            "enum": [1],
                                        },
                                        "paragraphs": {
                                            "type": "array",
                                            "description": "An array of paragraphs that belong to this section.",
                                            "minItems": 1,
                                            "items": {
                                                "type": "object",
                                                "properties": {
                                                    "paragraph_index": {
                                                        "type": "integer",
                                                        "description": "The original index of the paragraph in the source text, starting from 1.",
                                                        "minimum": 1,
                                                    },
                                                    "paragraph_title": {
                                                        "type": "string",
                                                        "description": "An engaging title for this specific paragraph.",
                                                    },
                                                    "title_level": {
                                                        "type": "integer",
                                                        "description": "The markdown heading level for the paragraph title.",
                                                        "enum": [2],
                                                    },
                                                },
                                                "required": [
                                                    "paragraph_index",
                                                    "paragraph_title",
                                                    "title_level",
                                                ],
                                                "additionalProperties": False,
                                            },
                                        },
                                    },
                                    "required": [
                                        "section_index",
                                        "section_title",
                                        "title_level",
                                        "paragraphs",
                                    ],
                                    "additionalProperties": False,
                                },
                            },
                            "key_concept": {
                                "type": "array",
                                "description": "A glossary of key terms with detailed explanations.",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "term": {"type": "string"},
                                        "explanation_points": {
                                            "type": "string",
                                        },
                                    },
                                    "required": ["term", "explanation_points"],
                                    "additionalProperties": False,
                                },
                            },
                        },
                        "required": ["sections", "key_concept"],
                        "additionalProperties": False,
                    },
                },
            },
        )
        return response.choices[0].message

    def insert_sections_and_paragraph_titles(
        self,
        original_md_path: Union[Path, str],
        json_path: Union[Path, str],
        output_md_path: Union[Path, str],
    ):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"JSON file not found at: {json_path}")
        if not os.path.exists(original_md_path):
            raise FileNotFoundError(
                f"Original Markdown file not found at: {original_md_path}"
            )
        with open(json_path, "r", encoding="utf-8") as f:
            api_response = json.load(f)
        if "content" in api_response and isinstance(api_response["content"], str):
            try:
                structured_data = json.loads(api_response["content"])
            except json.JSONDecodeError:
                raise ValueError(
                    f"The 'content' field in {json_path} is not valid JSON."
                )
        else:
            structured_data = api_response
        with open(original_md_path, "r", encoding="utf-8") as f:
            content = f.read()
        original_paragraphs = [p.strip() for p in content.split("\n\n") if p.strip()]
        new_content_parts = []
        for section in structured_data["sections"]:
            new_content_parts.append(f"# {section['section_title']}")
            for paragraph in section["paragraphs"]:
                new_content_parts.append(f"## {paragraph['paragraph_title']}")
                p_index = paragraph["paragraph_index"] - 1
                if 0 <= p_index < len(original_paragraphs):
                    new_content_parts.append(original_paragraphs[p_index])
                else:
                    raise ValueError(
                        f"Paragraph index {p_index + 1} is out of bounds. "
                        f"JSON and Markdown files might be mismatched."
                    )
        main_content = "\n\n".join(new_content_parts)
        final_markdown = main_content
        if output_md_path is None:
            base, _ = os.path.splitext(original_md_path)
            output_md_path = f"{base}.structured.md"

        with open(output_md_path, "w", encoding="utf-8") as f:
            f.write(final_markdown)

        print(f"✅ Successfully created structured markdown file at: {output_md_path}")
        return output_md_path


# if __name__ == "__main__":
# course_name_61a = "Structure and Interpretation of Computer Programs"
# course_name_294 = "Immersive Computing and Virtual Reality"
# file_path_61a = Path("/rag/file_conversion_router/test/18-Objects_1pp.md")
# file_path_294 = Path(
#     "/rag/file_conversion_router/test/Interaction Techniques Part 3 - Symbolic Input and Text Entry.md")
# pdf_md_structure = OpenAIChatMarkdownStructure(api_key,file_path_61a,"pdf", course_name_61a)
# video_md_structure = OpenAIChatMarkdownStructure(api_key,file_path_294,"mp4",course_name_294)
# output_path = pdf_md_structure._markdown_structure(pdf_md_structure)
# output_path = video_md_structure._markdown_structure()
