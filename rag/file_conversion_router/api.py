"""Public API for the file conversion router module.
"""

from pathlib import Path
from typing import Union

from rag.file_conversion_router.services.directory_service import process_folder
from rag.file_conversion_router.utils.logger import content_logger, set_log_file_path


def convert_directory(input_dir: Union[str, Path], output_dir: Union[str, Path]):
    """Convert all supported files in the given directory to Markdown format, to the specified output directory.

    Current supported file types:
    1. PDF
    2. Markdown (To clarify, this markdown includes additional tree structure of original markdown file)
    """
    set_log_file_path(content_logger, output_dir)
    process_folder(input_dir, output_dir, content_logger)


if __name__ == "__main__":
    convert_directory("test_in", "tests_out")
    convert_directory("test_in", "tests_out2")
