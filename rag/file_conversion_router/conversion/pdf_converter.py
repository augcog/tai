import subprocess
from pathlib import Path

from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.utils.hardware_detection import detect_gpu_setup


class PdfConverter(BaseConverter):
    def __init__(self, model_tag: str = "0.1.0-small", batch_size: int = 4):
        super().__init__()
        self.model_tag = model_tag
        self.batch_size = batch_size
        self.device_type, _ = detect_gpu_setup()  # Detect hardware configuration

        self._validate_parameters()

        self._logger.info(f"Using {self.device_type} on Torch")

    def _validate_parameters(self):
        """Validate model tag and batch size."""
        if not isinstance(self.batch_size, int) or self.batch_size <= 0:
            raise ValueError("Batch size must be a positive integer")
        # https://github.com/facebookresearch/nougat/issues/179#issuecomment-1831849650
        if self.device_type == "cpu":
            self.batch_size = 0
            self._logger.info("Forcing batch size to 0 for running on CPU")
        acceptable_models = ["0.1.0-small", "0.1.0-base"]
        if self.model_tag not in acceptable_models:
            raise ValueError(f"Model tag must be one of {acceptable_models}")

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Perform PDF to Markdown conversion using Nougat with the detected hardware configuration."""
        command = [
            "nougat",
            str(input_path),
            # nougat requires the argument output path to be a directory, not file, so we need to handle it here
            "-o",
            str(output_path.parent),
            "--no-skipping",
            "--model",
            self.model_tag,
            "--batchsize",
            str(self.batch_size),
        ]
        try:
            subprocess.run(command, check=True, capture_output=True, text=True)
            # Now change the file name of generated mmd file to align with the expected md file path from base converter
            output_mmd_path = output_path.parent / f"{input_path.stem}.mmd"
            # Rename it to `md` file
            output_mmd_path.rename(output_path)
        except subprocess.CalledProcessError as e:
            self._logger.error(f"Failed to execute conversion command: {e.stderr}")
            raise
