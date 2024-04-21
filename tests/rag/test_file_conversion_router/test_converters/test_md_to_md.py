# from pathlib import Path

# import pytest

# from tests.test_rag.conftest import load_test_cases_config
# from tests.test_rag.utils.utils import compare_files

#
# @pytest.mark.parametrize("input_path, expected_output_path", load_test_cases_config("unit_tests", "md_to_md"))
# def test_md_to_md_conversion(input_path: str, expected_output_path: str, tmp_path, md_to_md_converter):
#     input_path, expected_path = Path(input_path), Path(expected_output_path)
#     output_path = tmp_path / input_path.with_suffix(".mmd").name
#     md_to_md_converter.convert(input_path, output_path)
#     assert compare_files(expected_path, output_path), f"File conversion for {input_path} did not meet expectations."
