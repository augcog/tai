from pathlib import Path

import pytest

from rag.file_conversion_router.services.directory_service import process_folder
from tests.test_rag.conftest import load_test_cases_config
from tests.utils import compare_folders


@pytest.mark.parametrize(
    "input_folder, expected_output_folder",
    [
        *load_test_cases_config("integrated_tests", "plain_folder_3_pdfs"),
        *load_test_cases_config("integrated_tests", "nested_folder_pdf+md"),
    ],
)
def test_folder_conversion(input_folder: str, expected_output_folder: str, tmp_path):
    input_path = Path(input_folder)
    expected_path = Path(expected_output_folder)
    output_path = tmp_path / "output"
    process_folder(input_path, output_path)
    assert compare_folders(
        expected_path, output_path
    ), f"Folder conversion for {input_folder} did not meet expectations."
