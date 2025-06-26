# file: video_processor.py

import json
from pathlib import Path
from textwrap import dedent

from openai.types.chat import ChatCompletionMessage

# Import the base class from our other file
from rag.file_conversion_router.post_processing.MarkdownProcessor.BaseMarkdownProcessor import MarkdownStructureBase


class VideoMarkdownStructure(MarkdownStructureBase):
    """Structures markdown files originating from video transcripts (mp4, mp3)."""

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
                                        You are an expert AI assistant specializing in analyzing and structuring educational material. You will be given markdown content from a video in the course "{self.course_name}", from the file "{file_name}". The text is already divided into paragraphs.
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
                                        """)
                },
                {
                    "role": "user",
                    "content": f"{content} "
                }
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
                                        "section_index": {"type": "integer",
                                                          "description": "The sequential index of the section (e.g., 1, 2, 3).",
                                                          "minimum": 1},
                                        "section_title": {"type": "string",
                                                          "description": "A concise title for the entire section."},
                                        "title_level": {"type": "integer",
                                                        "description": "The markdown heading level for the section title.",
                                                        "enum": [1]},
                                        "paragraphs": {
                                            "type": "array",
                                            "description": "An array of paragraphs that belong to this section.",
                                            "minItems": 1,
                                            "items": {
                                                "type": "object",
                                                "properties": {
                                                    "paragraph_index": {"type": "integer",
                                                                        "description": "The original index of the paragraph in the source text, starting from 1.",
                                                                        "minimum": 1},
                                                    "paragraph_title": {"type": "string",
                                                                        "description": "An engaging title for this specific paragraph."},
                                                    "title_level": {"type": "integer",
                                                                    "description": "The markdown heading level for the paragraph title.",
                                                                    "enum": [2]}
                                                },
                                                "required": ["paragraph_index", "paragraph_title", "title_level"],
                                                "additionalProperties": False
                                            }
                                        }
                                    },
                                    "required": ["section_index", "section_title", "title_level", "paragraphs"],
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
                        "required": ["sections", "key_concept"],
                        "additionalProperties": False
                    }
                }
            },
        )
        return response.choices[0].message

    def _apply_structure_to_markdown(self, json_path: Path, output_path: Path):
        # ... (This method is identical to its previous version)
        # ... [omitted for brevity, copy from the previous answer] ...
        with open(json_path, 'r', encoding='utf-8') as f:
            api_response = json.load(f)
        structured_data = json.loads(api_response['content'])
        original_content = self.file_path.read_text(encoding='utf-8')
        original_paragraphs = [p.strip() for p in original_content.split('\n\n') if p.strip()]
        new_content_parts = []
        for section in structured_data['sections']:
            new_content_parts.append(f"# {section['section_title']}")
            for paragraph_info in section['paragraphs']:
                new_content_parts.append(f"## {paragraph_info['paragraph_title']}")
                p_index = paragraph_info['paragraph_index'] - 1
                if 0 <= p_index < len(original_paragraphs):
                    new_content_parts.append(original_paragraphs[p_index])
                else:
                    raise IndexError(f"Paragraph index {p_index + 1} from JSON is out of bounds.")
        final_markdown = "\n\n".join(new_content_parts)
        output_path.write_text(final_markdown, encoding='utf-8')
        print(f"✅ Successfully created structured markdown file at: {output_path}")