from pathlib import Path
from file_conversion_router.conversion.base_converter import BaseConverter


class MarkdownConverter(BaseConverter):
    def __init__(self, course_name, course_code, file_uuid: str = None):
        super().__init__(course_name, course_code,file_uuid)

    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        """Perform reStructuredText to Markdown conversion."""
        index_helper = {}
        output_path = output_path.with_suffix(".md")
        with open(input_path, "r", encoding="utf-8") as input_file:
            content = input_file.read()
            if not content:
                self._logger.warning(f"Input file {input_path} is empty.")
            lines = input_file.readlines()
            for i, line in enumerate(lines):
                if line.startswith("#"):
                    title = line.strip().lstrip("#").strip()
                    index_helper[title] = i + 1
            with open(output_path, "w", encoding="utf-8") as output_file:
                output_file.write(content)
            self.generate_index_helper(md=content)
        return output_path
