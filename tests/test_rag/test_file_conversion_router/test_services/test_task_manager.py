from pathlib import Path
from typing import List

import pytest

from rag.file_conversion_router.services.task_manager import schedule_conversion
from tests.test_rag.conftest import load_test_cases_config
from tests.utils import compare_files


@pytest.mark.parametrize(
    "input_path, expected_output_paths",
    load_test_cases_config("unit_tests", "pdf_converter"),
)
def test_pdf_converter_on_task_manager(
    input_path: str, expected_output_paths: List[str], tmp_path, pdf_converter
):
    input_path = Path(input_path)
    expected_paths = [
        Path(expected_output_path) for expected_output_path in expected_output_paths
    ]
    output_folder = tmp_path / input_path.stem

    # Schedule the conversion using the asynchronous scheduler
    future = schedule_conversion(pdf_converter.convert, input_path, output_folder)

    # Wait for the task to complete and check for exceptions
    try:
        future.result()  # Ensures that the conversion has finished before moving forward
    except Exception as e:
        pytest.fail(f"Conversion failed with an exception: {e}")

    # Now that the task is complete, check the file against the expected output
    for idx, suffix in enumerate([".md", ".pkl"]):
        output_file_path = output_folder / f"{input_path.stem}{suffix}"
        assert output_file_path.exists(), f"File {output_file_path} does not exist."
        assert compare_files(
            expected_paths[idx], output_file_path
        ), f"File conversion for {input_path} did not meet expectations."
