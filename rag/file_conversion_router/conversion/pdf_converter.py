import os
import re
import subprocess
from pathlib import Path
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.services.tai_MinerU_service.api import convert_pdf_to_md_by_MinerU

class PdfConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)
        self.available_tools = ["nougat", "MinerU"]

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
        image_link_pattern = r'!\[.*?\]\(.*?\)'
        # Remove all image links
        return re.sub(image_link_pattern, '', text)


    def clean_markdown_content(self, markdown_path):
        with open(markdown_path, 'r', encoding='utf-8') as file:
            content = file.read()
        cleaned_content = self.remove_image_links(content)
        with open(markdown_path, 'w', encoding='utf-8') as file:
            file.write(cleaned_content)

    def validate_tool(self, tool_name):
        """
        Validate if the tool is supported, raise an error if not.
        """
        if not self.is_tool_supported(tool_name):
            raise ValueError(f"Tool '{tool_name}' is not supported. Available tools: {', '.join(self.available_tools)}")

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path, conversion_method: str = "MinerU") -> Path:
        # """Perform PDF to Markdown conversion using Nougat with the detected hardware configuration."""
        self.validate_tool(conversion_method)
        temp_dir_path = output_path.parent

        # Create the directory if it doesn't exist
        if not temp_dir_path.exists():
            os.makedirs(temp_dir_path)
        if conversion_method == "MinerU":
            new_output_path = output_path.with_suffix('')
            convert_pdf_to_md_by_MinerU(input_path, new_output_path)
            base_name = input_path.stem  # e.g., "07-Function_Examples_1pp"
            md_file_path = new_output_path.parent / f"{base_name}.md"
            if md_file_path.exists():
                print(f"Markdown file found: {md_file_path}")
            else:
                raise FileNotFoundError(f"Markdown file not found: {md_file_path}")
            target = md_file_path
            self.clean_markdown_content(target)
        return target
