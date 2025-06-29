from pathlib import Path

from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page
from rst_to_myst import rst_to_myst
import yaml


class RstConverter(BaseConverter):
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
            content = rst_to_myst(input_file.read())
            output_file.write(content.text)
        return output_path

    # def _to_page(self, input_path: Path, output_path: Path) -> Page:
    #     """Perform Markdown to Page conversion."""

    #     output_path.parent.mkdir(parents = True, exist_ok = True)

    #     parent = input_path.parent
    #     self._to_markdown(input_path, output_path)
    #     stem = input_path.stem
    #     filetype = input_path.suffix.split(".")[1]

    #     with open(input_path, "r") as input_file:
    #         text = input_file.read()
    #     metadata = parent / (stem+"_metadata.yaml")

    #     with open(metadata, "r") as metadata_file:
    #         metadata_content = yaml.safe_load(metadata_file)

    #     url = metadata_content["URL"]
    #     page = Page(pagename=stem, content={'text': text}, filetype=filetype, page_url=url)
    #     return page
