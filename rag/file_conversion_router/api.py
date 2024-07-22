"""Public API for the file conversion router module.
"""

from pathlib import Path
from typing import Union

from rag.file_conversion_router.services.directory_service import process_folder


def convert_directory(input_dir: Union[str, Path], output_dir: Union[str, Path]):
    """Convert all supported files in the given directory to Markdown format, to the specified output directory.

    Current supported file types:
    1. PDF
    2. Markdown (To clarify, this markdown includes additional tree structure of original markdown file)
    """
    process_folder(input_dir, output_dir)

if __name__ == "__main__": 
    convert_directory("tests\\test_rag\\data\\integrated_tests\\input_folder2_nested_folder_pdf+md\\mds\\mds_1", "tests\\test_rag\\data\\integrated_tests\\expected_output_folder2_nested_folder_pdf+md\\mds\\mds_1\\section-0-brief-python-refresher")