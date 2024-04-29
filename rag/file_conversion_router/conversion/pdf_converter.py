import subprocess
from pathlib import Path
from .base_converter import BaseConverter
import os
from rag.file_conversion_router.utils.time_measure import Timer
from rag.file_conversion_router.utils.hardware_detection import detect_gpu_setup


class PdfToMdConverter(BaseConverter):
    def __init__(self, model_tag: str = "0.1.0-small", batch_size: int = 4):
        super().__init__()
        self.model_tag = model_tag
        self.batch_size = batch_size
        self.device_type, _ = detect_gpu_setup()  # Detect hardware configuration
        self.validate_parameters()

    def validate_parameters(self):
        """Validate model tag and batch size."""
        if not isinstance(self.batch_size, int) or self.batch_size <= 0:
            raise ValueError("Batch size must be a positive integer")
        acceptable_models = ["0.1.0-small", "0.1.0-base"]
        if self.model_tag not in acceptable_models:
            raise ValueError(f"Model tag must be one of {acceptable_models}")

    def perform_conversion(self, input_path: Path, output_path: Path) -> None:
        """Perform PDF to Markdown conversion using Nougat with the detected hardware configuration."""
        if not input_path.exists():
            self.logger.error(f"The file {input_path} does not exist.")
            raise FileNotFoundError(f"The file {input_path} does not exist.")

        output_path = output_path.parent
        self.logger.info(f"Starting conversion from PDF to Markdown for {input_path} on {self.device_type.upper()}")

        if self.device_type == "cuda":
            os.environ['CUDA_VISIBLE_DEVICES'] = '0'
        elif self.device_type == "mps":
            os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'
        else:
            os.environ['CUDA_VISIBLE_DEVICES'] = ''

        command = [
            "nougat",
            str(input_path),
            "-o", str(output_path),
            "--no-skipping",
            "--model", self.model_tag,
            "--batchsize", str(self.batch_size)
        ]

        with Timer() as timer:
            try:
                subprocess.run(command, check=True, capture_output=True, text=True)
            except subprocess.CalledProcessError as e:
                self.logger.error(f"Failed to execute conversion command: {e.stderr}")
                raise

        if timer.interval is not None:
            self.logger.info(f"Succesfully executed conversion command for {input_path} to {output_path} "
                             f"in {timer.interval:.2f} seconds.")
