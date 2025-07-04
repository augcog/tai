from typing import List

import pytest

from tests.test_rag.conftest import (
    helper_unit_test_on_converter,
    load_test_cases_config,
)


@pytest.mark.parametrize(
    "input_path, expected_output_paths",
    load_test_cases_config("unit_tests", "pdf_converter"),
)
def test_pdf_conversion(
    input_path: str, expected_output_paths: List[str], tmp_path, pdf_converter
):
    helper_unit_test_on_converter(
        input_path, expected_output_paths, tmp_path, pdf_converter
    )
