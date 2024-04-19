import subprocess
from pathlib import Path

from .base_converter import BaseConverter


class PdfToMdConverter(BaseConverter):
    def perform_conversion(self, input_path: Path, output_path: Path) -> None:
        """Convert a PDF file to Markdown format using a specific tool."""
        # This is because `nougat` generates a file to the output directory
        output_path = output_path.parent
        self.logger.info(f"Starting conversion from PDF to Markdown for {input_path}")
        command = ["nougat", str(input_path), "-o", str(output_path)]
        try:
            subprocess.run(command, check=True)
            self.logger.info(f"Successfully executed conversion command for {input_path} to {output_path}")
        except subprocess.CalledProcessError as e:
            self.logger.error(f"Failed to execute conversion command: {e}")
            raise
