# file: pdf_processor.py

import json
import re
from pathlib import Path
from textwrap import dedent

from openai.types.chat import ChatCompletionMessage

from rag.file_conversion_router.post_processing.MarkdownProcessor.BaseMarkdownProcessor import MarkdownStructureBase


class PdfMarkdownStructure(MarkdownStructureBase):
    """Structures markdown files originating from PDFs."""

    def _preprocess(self):
        self._remove_redundant_title()

    def _get_structured_content(self) -> ChatCompletionMessage:
        with open(self.file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        if not content.strip(): raise ValueError("The content is empty or not properly formatted.")
        file_name = self.file_path.name
        response = self.client.chat.completions.create(
            model="gpt-4.1",
            messages=[
                {
                    "role": "system",
                    "content": dedent(f"""
                    You are an expert AI assistant for structuring educational material. You will be given markdown content from the file "{file_name}" for the course "{self.course_name}".
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
                """)
                },
                {
                    "role": "user",
                    "content": f"{content}"
                }
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
                                        "title": {"type": "string",
                                                  "description": "The clean text of the title, without any '#' characters."},

                                        "level_of_title": {"type": "integer",
                                                           "description": "The inferred hierarchy level (e.g., 1, 2, 3)."}
                                    },
                                    "required": ["title", "level_of_title"],
                                    "additionalProperties": False
                                }
                            },
                            "key_concept": {
                                "type": "array",
                                "description": "A glossary of key terms with detailed explanations.",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "term": {"type": "string"},
                                        "explanation_points": {"type": "string", }
                                    },
                                    "required": ["term", "explanation_points"],
                                    "additionalProperties": False
                                }
                            }
                        },
                        "required": ["titles_with_levels", "key_concept"],
                        "additionalProperties": False
                    },
                }
            },
        )
        return response.choices[0].message

    def _apply_structure_to_markdown(self, json_path: Path, output_path: Path):
        # ... (This method is identical to its previous version)
        # ... [omitted for brevity, copy from the previous answer] ...
        with open(json_path, 'r', encoding='utf-8') as f:
            raw_response = json.load(f)
        structured_data = json.loads(raw_response['content'])
        mapping_list = structured_data.get('titles_with_levels')
        if not mapping_list: raise KeyError("Could not find 'titles_with_levels' in the JSON response.")
        title_level_map = {item['title'].strip(): int(item['level_of_title']) for item in mapping_list}
        lines = self.file_path.read_text(encoding='utf-8').splitlines(keepends=True)
        title_pattern = re.compile(r'^(?P<hashes>#+)\s*(?P<title>.+?)\s*$')
        new_lines = []
        for line in lines:
            match = title_pattern.match(line)
            if match:
                raw_title = match.group('title').strip()
                if raw_title in title_level_map:
                    new_level = title_level_map[raw_title]
                    new_lines.append(f"{'#' * new_level} {raw_title}\n")
                    continue
            new_lines.append(line)
        output_path.write_text(''.join(new_lines), encoding='utf-8')
        print(f"✅ Rewrote markdown with corrected title levels to: {output_path}")

    def _remove_redundant_title(self) -> bool:
        # ... (This method is identical to its previous version)
        # ... [omitted for brevity, copy from the previous answer] ...
        normalized_filename = self.file_path.stem.lower().replace('-', ' ').replace('_', ' ')
        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        except FileNotFoundError:
            return False
        if not lines: return False
        first_line = lines[0]
        if first_line.startswith('# '):
            title_text = first_line.lstrip('# ').strip()
            if normalized_filename == title_text.lower():
                print(f"Found and removing redundant title in '{self.file_path.name}'.")
                remaining_lines = lines[1:]
                while remaining_lines and remaining_lines[0].strip() == '': remaining_lines.pop(0)
                with open(self.file_path, 'w', encoding='utf-8') as f:
                    f.write("".join(remaining_lines))
                return True
        print(f"No redundant title found in '{self.file_path.name}'.")
        return False