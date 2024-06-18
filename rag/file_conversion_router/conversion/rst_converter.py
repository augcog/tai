from pathlib import Path

from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page
from rag.file_conversion_router.classes.chunk import Chunk
from rst_to_myst import rst_to_myst
import yaml

class RstConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    def _to_markdown(self, input_path: Path) -> Path:
        """Perform reStructuredText to Markdown conversion.

        Arguments:
        input_path -- Path to the input rst file.
        output_folder -- Path to the folder where the output md file will be saved.
        """
        # Ensure the output folder exists

        # Determine the output path
        output_path = input_path / (input_path.stem + ".md")

        with open(input_path, "r") as input_file, open(output_path, "w") as output_file:
            content = str(rst_to_myst(input_file.read()))
            output_file.write(content)
        return output_path

    def _to_page(self, input_path: Path, output_path: Path, url) -> Page:
        """Perform Markdown to Page conversion."""
        input_path = self._to_markdown(input_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        # parent = input_path.parent
        stem = input_path.stem
        filetype = input_path.suffix.split(".")[1]
        with open(input_path, "r") as input_file:
            text = input_file.read()
        metadata = output_path / (stem+"_metadata.yaml")
        with open(metadata, "r") as metadata_file:
            metadata_content = yaml.safe_load(metadata_file)
        url = metadata_content["URL"]
        return Page(content={'text': text}, filetype=filetype, page_url=url)

    def _to_chunk(self, page: Page) -> list[Chunk]:
        """Perform Page to Chunk conversion."""
        page.page_seperate_to_segments()
        page.tree_print()
        return page.tree_segments_to_chunks()