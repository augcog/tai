from pathlib import Path

from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page

class MarkdownConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Perform Markdown to Markdown conversion.
        Current implementation is to output the same content as input.
        """
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(input_path, "r") as input_file, open(output_path, "w") as output_file:
            content = input_file.read()
            output_file.write(content)

    def _to_page(self, input_path: Path, output_path: Path, url) -> None:
        """Perform Markdown to Page conversion."""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        stem = input_path.stem
        with open(input_path, "r") as input_file:
            text = input_file.read()

        page = Page(content={'text': text, "type": "md"}, filetype="md", page_url="url")
        raise NotImplementedError("Markdown to Page conversion is not supported.")
