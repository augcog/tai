from pathlib import Path

from rst_to_myst import rst_to_myst

from file_conversion_router.classes.page import Page
from file_conversion_router.conversion.base_converter import BaseConverter


class RstConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)

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
    def title_to_index(self, md_path: Path) -> list[dict]:
        return []