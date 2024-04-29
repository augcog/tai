from pathlib import Path
from rag.file_conversion_router.conversion.base_converter import BaseConverter


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
