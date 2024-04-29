from pathlib import Path

import pytest
from typing import List

from tests.rag.conftest import load_test_cases_config, helper_unit_test_on_converter
from tests.rag.utils import compare_files
import os


@pytest.mark.parametrize(
    "input_path, expected_output_paths",
    load_test_cases_config("unit_tests", "md_converter"),
)
def test_pdf_to_md_conversion(input_path: str, expected_output_paths: List[str], tmp_path, pdf_converter):
    helper_unit_test_on_converter(input_path, expected_output_paths, tmp_path, pdf_converter)
