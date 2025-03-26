from pathlib import Path
import nbformat
from nbconvert import MarkdownExporter

from rag.file_convrsion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page


class NotebookConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        output_path = output_path.with_suffix(".md")
        
        with open(input_path, "r") as input_file, open(output_path, "w") as output_file:
            content = nbformat.read(input_file, as_version=4)
            markdown_converter = MarkdownExporter()
            (markdown_content, resources) = markdown_converter.from_notebook_node(content)
            output_file.write(self._post_process_markdown(markdown_content))
        return output_path

    def _post_process_markdown(self, markdown_content: str) -> str:
        lines = markdown_content.split("\n")[1:] # first line is the title of the course section

        processed_lines = []
        for i, line in enumerate(lines):
            if i == 1: # convert lecture title to h1
                processed_lines.append(f"# {line.lstrip('#').strip()}")
            elif line.startswith("#"): # convert all other heading down one level
                processed_lines.append(f"#{line.strip()}")
            else:
                processed_lines.append(line.strip()) 

        return "\n".join(processed_lines) 
