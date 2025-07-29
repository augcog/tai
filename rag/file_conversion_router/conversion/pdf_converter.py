from file_conversion_router.conversion.base_converter import BaseConverter

from file_conversion_router.services.tai_MinerU_service.api import (
    convert_pdf_to_md_by_MinerU,
)
from dotenv import load_dotenv
import os
from openai import OpenAI
from pathlib import Path
import json
import re
from textwrap import dedent


class PdfConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)
        self.available_tools = ["MinerU"]
        self.index_helper = None
        self.file_name = ""

    def is_tool_supported(self, tool_name):
        """
        Check if a tool is supported.
        """
        return tool_name in self.available_tools

    def remove_image_links(self, text):
        """
        Remove image links from the text.
        """
        # Regular expression to match image links
        image_link_pattern = r"!\[.*?\]\(.*?\)"
        # Remove all image links
        return re.sub(image_link_pattern, "", text)

    def clean_markdown_content(self, markdown_path):
        with open(markdown_path, "r", encoding="utf-8") as file:
            content = file.read()
        cleaned_content = self.remove_image_links(content)
        with open(markdown_path, "w", encoding="utf-8") as file:
            file.write(cleaned_content)

    def validate_tool(self, tool_name):
        """
        Validate if the tool is supported, raise an error if not.
        """
        if not self.is_tool_supported(tool_name):
            raise ValueError(
                f"Tool '{tool_name}' is not supported. Available tools: {', '.join(self.available_tools)}"
            )

    # Override
    def _to_markdown(
        self, input_path: Path, output_path: Path, conversion_method: str = "MinerU"
    ) -> Path:
        self.file_name = input_path.name
        self.validate_tool(conversion_method)
        temp_dir_path = output_path.parent

        # Create the directory if it doesn't exist
        if not temp_dir_path.exists():
            os.makedirs(temp_dir_path)
        if conversion_method == "MinerU":
            new_output_path = output_path.with_suffix("")
            md_file_path = convert_pdf_to_md_by_MinerU(input_path, new_output_path)

            if md_file_path.exists():
                print(f"Markdown file found: {md_file_path}")
            else:
                raise FileNotFoundError(f"Markdown file not found: {md_file_path}")
            # Set the target to this markdown path
            target = md_file_path
            self.clean_markdown_content(target)
            json_file_path = md_file_path.with_name(f"{md_file_path.stem}_content_list.json")
            with open(json_file_path, "r", encoding="utf-8") as f_json:
                data = json.load(f_json)
            self.generate_index_helper(data)
            return target

    def generate_index_helper(self, data):
        self.index_helper  = []
        for item in data:
            if item.get('text_level') == 1:
                title = item['text'].strip()
                pattern = r'^\s*ROAR ACADEMY EXERCISES\s*$'
                if re.match(pattern, title):
                    continue
                page_index = item['page_idx'] + 1  # Convert to 1-based indexing
                self.index_helper.append({title: page_index})

    def get_structured_content_for_pdf(self, md_content):
        """
        Get structured content from OpenAI GPT.
        """
        load_dotenv()
        openai_api_key = os.getenv("OPENAI_API_KEY")
        client = OpenAI(api_key=openai_api_key)
        if not md_content.strip():
            raise ValueError("The content is empty or not properly formatted.")
        title_list = [key for d in self.index_helper for key in d.keys()]
        response = client.chat.completions.create(
            model="gpt-4.1",
            temperature=0.1,
            messages=[
                {
                    "role": "system",
                    "content": dedent(
                        f""" You are an expert AI assistant for structuring educational material. You will 
                            be given markdown content from the file "{self.file_name}" for the course "{self.course_name}". Your task is to 
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
            response_format={
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
        )
        messages = response.choices[0].message
        data = messages.content
        content_dict = json.loads(data)
        return content_dict

    def remove_redundant_title(self,md_content: str, file_name: str):
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

    def fix_title_levels(self, mapping_list):
        """
        Fix title levels in the mapping list to ensure they are sequential.
        """
        last_level = 0
        for i in range(len(mapping_list) - 1):
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

    def get_structured_content_from_gpt(self, md_content, content_dict):
        """
        Use GPT result to get structured content.
        """
        mapping_list = content_dict.get("titles_with_levels")
        mapping_list = self.fix_title_levels(mapping_list)
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
