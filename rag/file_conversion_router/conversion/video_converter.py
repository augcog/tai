from pathlib import Path

from rag.file_conversion_router.conversion.base_converter import BaseConverter


class VideoConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Perform Video to Markdown conversion."""
        # TODO: Implement Video to Markdown conversion by integrating our existing video processing pipeline.
        raise NotImplementedError("Video to Markdown conversion is not supported.")
