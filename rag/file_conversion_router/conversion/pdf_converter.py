import subprocess
from pathlib import Path

from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.utils.hardware_detection import detect_gpu_setup
from rag.file_conversion_router.classes.page import Page
from rag.file_conversion_router.classes.chunk import Chunk
import yaml

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
        if not isinstance(self.batch_size, int) or self.batch_size < 0:
            raise ValueError("Batch size must be a non-negative integer.")
        # https://github.com/facebookresearch/nougat/issues/179#issuecomment-1831849650
        if self.device_type == "cpu":
            self.batch_size = 0
            self._logger.info("Forcing batch size to 0 for running on CPU")
        acceptable_models = ["0.1.0-small", "0.1.0-base"]
        if self.model_tag not in acceptable_models:
            raise ValueError(f"Model tag must be one of {acceptable_models}")

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        # """Perform PDF to Markdown conversion using Nougat with the detected hardware configuration."""
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
            result = subprocess.run(command, check=False, capture_output=True, text=True)
            self._logger.info(f"Output: {result.stdout}")
            self._logger.info(f"Errors: {result.stderr}")
            if result.returncode != 0:
                self._logger.error(f"Command exited with a non-zero status: {result.returncode}")
        except Exception as e:
            self._logger.error(f"An error occurred: {str(e)}")
            raise

        # Now change the file name of generated mmd file to align with the expected md file path from base converter
        output_mmd_path = output_path.with_suffix(".mmd")
        # Rename it to `md` file
        target = output_path.with_suffix(".md")
        output_mmd_path.rename(target)
        print(output_mmd_path)
        return target
        #
        # except Exception as e:
        #     self._logger.error(f"An error occurred {str(e)})")
        #     raise

    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        """Perform Markdown to Page conversion."""
        try:
            input_path = self._to_markdown(input_path, output_path)
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
