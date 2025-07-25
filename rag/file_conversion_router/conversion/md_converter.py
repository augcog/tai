from pathlib import Path

from file_conversion_router.classes.page import Page
from file_conversion_router.conversion.base_converter import BaseConverter


class MarkdownConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)

    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        """Perform reStructuredText to Markdown conversion."""
        output_path = output_path.with_suffix(".md")
        try:
            with open(input_path, "r", encoding="utf-8") as input_file:
                content = input_file.read()
                if not content:
                    self._logger.warning(f"Input file {input_path} is empty.")
                with open(output_path, "w", encoding="utf-8") as output_file:
                    output_file.write(content)
                self.generate_index_helper(content)
        except Exception as e:
            self._logger.error(f"Error reading file {input_path}: {str(e)}")
        return output_path

    def title_to_index(self, md_path: Path) -> dict:
        """
        Extract titles from the Markdown file and create an index helper.

        Args:
            md_path (Path): Path to the Markdown file.

        Returns:
            list[dict]: List of dictionaries with titles as keys and their indices as values.
        """
        index_helper = {}
        try:
            with open(md_path, "r", encoding="utf-8") as file:
                lines = file.readlines()
                for i, line in enumerate(lines):
                    if line.startswith("#"):
                        title = line.strip().lstrip("#").strip()
                        index_helper[title] = i + 1
        except Exception as e:
            self._logger.error(f"Error reading Markdown file {md_path}: {str(e)}")
        return index_helper
