import logging
from pathlib import Path

from file_conversion_router.classes.page import Page
from file_conversion_router.conversion.base_converter import BaseConverter


class PythonConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        """Converts a Python file to a Markdown file by formatting it as a code block."""

        output_path = output_path.with_suffix(".md")
        title = input_path.stem

        with open(input_path, "r") as input_file, open(output_path, "w") as output_file:
            content = input_file.read()
            if not content.strip():
                logging.warning(f"File {input_path} is empty, skipping conversion.")
                return None

            # Write filename as a Markdown H1 title, then the code block
            markdown_content = f"# {title}\n\n```python\n{content}\n```"
            output_file.write(markdown_content)
        self.generate_index_helper(markdown_content)
        return output_path
