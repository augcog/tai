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
    print("API module is running")

    process_folder(input_dir, output_dir)
if __name__ == '__main__' :
    # convert_directory("/Users/yyk956614/tai/output_tmp/input", "/Users/yyk956614/tai/output_tmp/expected_output/debug")
    convert_directory("/home/roar-tai-1/Yikang_testing/tai/tests/test_rag/data/integrated_tests/input_folder1_plain_folder_3_pdfs", "/home/roar-tai-1/Yikang_testing/tai/tests/test_rag/data/integrated_tests/expected_output_folder1_plain_folder_3_pdfs")
    convert_directory("/home/roar-tai-1/Yikang_testing/tai/tests/test_rag/data/integrated_tests/input_folder2_nested_folder_pdf+md", "/home/roar-tai-1/Yikang_testing/tai/tests/test_rag/data/integrated_tests/expected_output_folder2_nested_folder_pdf+md")
    convert_directory("/home/roar-tai-1/Yikang_testing/tai/tests/test_rag/data/unit_tests/pdf/input", "/home/roar-tai-1/Yikang_testing/tai/tests/test_rag/data/unit_tests/pdf/expected_output")
    convert_directory("/home/roar-tai-1/Yikang_testing/tai/tests/test_rag/data/unit_tests/md/input", "/home/roar-tai-1/Yikang_testing/tai/tests/test_rag/data/unit_tests/md/expected_output")
    # convert_directory("/Users/yyk956614/tai/tests/test_rag/data/integrated_tests/input_folder2_nested_folder_pdf+md", "/Users/yyk956614/tai/tests/test_rag/data/integrated_tests/expected_output_folder2_nested_folder_pdf+md")