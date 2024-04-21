from pathlib import Path

import pytest

from rag.file_conversion_router.services.directory_service import process_folder
from tests.rag.conftest import load_test_cases_config
from tests.rag.utils import compare_folders


@pytest.mark.parametrize(
    "input_folder, expected_output_folder",
    load_test_cases_config("integrated_tests", "one_plain_folder_and_to_md+to_pdf"),
)
def test_folder_conversion(input_folder: str, expected_output_folder: str, tmp_path):
    input_path = Path(input_folder)
    expected_path = Path(expected_output_folder)
    output_path = tmp_path / "output"
    process_folder(input_path, output_path)
    assert compare_folders(
        expected_path, output_path
    ), f"Folder conversion for {input_folder} did not meet expectations."
