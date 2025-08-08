from file_conversion_router.conversion.base_converter import BaseConverter
from pathlib import Path
import json
import re

class TxtConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)
        self.index_helper = None
        self.file_name = ""

    def _to_markdown(self, input_path: Path, output_path: Path):
        """
        Convert a text file to markdown format.

        Args:
            input_path (Path): Path to the input text file.
            output_path (Path): Path where the markdown file will be saved.

        Returns: output_path
        """
        self.file_name = input_path.name
        # Read the content of the text file
        with open(input_path, "r", encoding="utf-8") as file:
            content = file.read()
        # Write the content to the markdown file
        with open(output_path, "w", encoding="utf-8") as md_file:
            md_file.write(content)
        # Generate index helper if needed
        self.generate_index_helper(content)
        return output_path