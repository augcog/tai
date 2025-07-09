import os
import re
from pathlib import Path
import json
import yaml
from file_conversion_router.conversion.base_converter import BaseConverter

from file_conversion_router.services.tai_MinerU_service.api import (
    convert_pdf_to_md_by_MinerU,
)


class PdfConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)
        self.available_tools = ["MinerU"]
        self.index_helper = None

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



