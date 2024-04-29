from pathlib import Path

from rag.file_conversion_router.conversion.base_converter import BaseConverter


class PdfConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Perform PDF to Markdown conversion using Nougat with the detected hardware configuration."""
        raise NotImplementedError("This method is not implemented yet.")
