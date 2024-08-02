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
    # convert_directory("tests\\test_rag\\data\\integrated_tests\\input_folder1_plain_folder_3_pdfs", "tests\\test_rag\\data\\integrated_tests\\expected_output_folder1_plain_folder_3_pdfs")
    # convert_directory("tests\\test_rag\\data\\integrated_tests\\input_folder2_nested_folder_pdf+md", "tests\\test_rag\\data\\integrated_tests\\expected_output_folder2_nested_folder_pdf+md")
    # convert_directory("tests\\test_rag\\data\\unit_tests\\md\\input", "tests\\test_rag\\data\\unit_tests\\md\\expected_output")
    # convert_directory("tests\\test_rag\\data\\unit_tests\\pdf\\input", "tests\\test_rag\\data\\unit_tests\\pdf\\expected_output")
    # convert_directory("output_tmp\input", "output_tmp\expected_output")