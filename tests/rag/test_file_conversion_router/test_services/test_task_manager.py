from pathlib import Path

import pytest

from rag.file_conversion_router.services.task_manager import schedule_conversion
from tests.rag.conftest import load_test_cases_config
from tests.rag.utils import compare_files


@pytest.mark.parametrize(
    "input_path, expected_output_path",
    load_test_cases_config("unit_tests", "pdf_to_md"),
)
def test_pdf_to_md_conversion(input_path: str, expected_output_path: str, tmp_path, pdf_to_md_converter):
    input_path, expected_path = Path(input_path), Path(expected_output_path)
    output_path = tmp_path / input_path.with_suffix(".mmd").name

    # Schedule the conversion using the asynchronous scheduler
    future = schedule_conversion(pdf_to_md_converter.convert, input_path, output_path)

    # Wait for the task to complete and check for exceptions
    try:
        future.result()  # Ensures that the conversion has finished before moving forward
    except Exception as e:
        pytest.fail(f"Conversion failed with an exception: {e}")

    # Now that the task is complete, check the file against the expected output
    matches_expectation = compare_files(expected_path, output_path)
    assert matches_expectation, f"File conversion for {input_path} did not meet expectations."
