from pathlib import Path
import re
import nbformat
from nbconvert import MarkdownExporter
import yaml
from file_conversion_router.classes.page import Page
from file_conversion_router.conversion.base_converter import BaseConverter


class NotebookConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)
        self.index_helper = [{}]

    def extract_all_markdown_titles(self, content):
        """
        Extract ALL possible titles from markdown content
        Returns a list of all found titles
        """
        if not content.strip():
            return []

        titles = []
        header_matches = re.findall(r'^\s*(#+)\s+(.+)\s*$', content, re.MULTILINE)
        for level, title in header_matches:
            clean_title = title.strip().lstrip('*').strip().rstrip('*').strip()
            if clean_title:
                titles.append(clean_title)

        star_matches = re.findall(r'^\s*\*+(.+)\*+\s*$', content, re.MULTILINE)
        for title in star_matches:
            clean_title = title.strip(' #*')
            if clean_title:
                titles.append(clean_title)
        return titles


    def generate_index_helper(self, notebook_content, markdown_content):
        self.index_helper = []
        for i, cell in enumerate(notebook_content.cells):
            titles = self.extract_all_markdown_titles(cell.source)
            for title in titles:
                self.index_helper.append({title: i + 1})

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        output_path = output_path.with_suffix(".md")

        with open(input_path, "r") as input_file, open(output_path, "w") as output_file:
            content = nbformat.read(input_file, as_version=4)
            markdown_converter = MarkdownExporter()
            (markdown_content, resources) = markdown_converter.from_notebook_node(
                content
            )
            self.generate_index_helper(content,markdown_content)
            output_file.write(self._post_process_markdown(markdown_content))
        return output_path

    def _post_process_markdown(self, markdown_content: str) -> str:
        lines = markdown_content.split("\n")[
            1:
        ]  # first line is the title of the course section

        processed_lines = []
        for i, line in enumerate(lines):
            if i == 1:  # convert lecture title to h1
                processed_lines.append(f"# {line.lstrip('#').strip()}")
            elif line.startswith("#"):  # convert all other heading down one level
                processed_lines.append(f"#{line.strip()}")
            else:
                processed_lines.append(line.strip())
        return "\n".join(processed_lines)
