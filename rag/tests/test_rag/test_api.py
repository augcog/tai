from pathlib import Path

import pytest

from file_conversion_router.api import convert_directory
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
    convert_directory(
        course_name="test",
        course_id="test",
        input_path=input_path,
        output_path=output_path,
    )
    assert compare_folders(expected_path, output_path), (
        f"Folder conversion for {input_folder} did not meet expectations."
    )
