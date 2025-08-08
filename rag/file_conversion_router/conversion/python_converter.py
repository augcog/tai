import logging
import re
from pathlib import Path
from file_conversion_router.conversion.base_converter import BaseConverter


class PythonConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)

    def _filter_pythontutor_links(self, content: str) -> str:
        """Remove lines that contain pythontutor.com links."""
        lines = content.split('\n')
        filtered_lines = []

        for line in lines:
            # Check if line is a comment containing pythontutor.com
            stripped_line = line.strip()
            if stripped_line.startswith('#') and 'pythontutor.com' in stripped_line:
                continue  # Skip this line
            filtered_lines.append(line)

        return '\n'.join(filtered_lines)

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

            # Filter out pythontutor links
            filtered_content = self._filter_pythontutor_links(content)

            # Write filename as a Markdown H1 title, then the code block
            markdown_content = f"# {title}\n\n```python\n{filtered_content}\n```"
            output_file.write(markdown_content)

        self.generate_index_helper(md=markdown_content)
        return output_path