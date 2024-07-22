from pathlib import Path
import yaml
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page
from rag.file_conversion_router.classes.chunk import Chunk

class MarkdownConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        """Perform reStructuredText to Markdown conversion.

        Arguments:
        input_path -- Path to the input rst file.
        output_folder -- Path to the folder where the output md file will be saved.
        """
        # Ensure the output folder exists
        # Determine the output path

        output_path = output_path.with_suffix(".md")
        with open(input_path, "r") as input_file, open(output_path, "w") as output_file:
            content = input_file.read()
            output_file.write(content)
        return output_path
    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        """Perform Markdown to Page conversion."""
        try:
            self._to_markdown(input_path, output_path)
        except Exception as e:
            self._logger.error(f"An error occurred during markdown conversion: {str(e)}")
            raise

        output_path.parent.mkdir(parents=True, exist_ok=True)

        filetype = input_path.suffix.lstrip('.')
        with open(input_path, "r") as input_file:
            text = input_file.read()

        metadata_path = input_path.with_name(f"{input_path.stem}_metadata.yaml")
        metadata_content = self._read_metadata(metadata_path)
        url = metadata_content.get("URL")
        return Page(pagename=input_path.stem, content={'text': text}, filetype=filetype, page_url=url)

