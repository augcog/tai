import yaml
from pathlib import Path
import pytest
from typing import Dict, List

# from rag.file_conversion_router.conversion.md_to_md import MdToMdConverter
from rag.file_conversion_router.conversion.pdf_to_md import PdfToMdConverter


TEST_CASES_CONFIG = None


def load_test_cases_config(*keys):
    """Load and return configurations for specified test cases from a YAML file.

    Each configuration entry should be formatted as a dictionary and the result is
    a list of tuples extracted from these dictionaries. The YAML file is loaded only once.

    Args:
        *keys (str): Variable length argument list of keys to access nested dictionary values.

    Returns:
        list: A list of tuples containing the values from the specified configuration entries.

    Raises:
        ValueError: If no keys are provided.

    Example:
    >>> (load_test_cases_config('unit_tests', 'example_format') ==
    ... [
    ... ('data/unit_tests/example_format/input/example_input.pdf',
    ...  'data/unit_tests/example_format/expected_output/example_output.mmd',
    ...  ['+example_diff', '-example_diff2']),
    ...  ('data/unit_tests/example_format/input/example_input_2.pdf',
    ...   'data/unit_tests/example_format/expected_output/example_output_2.mmd',
    ...  None),
    ... ]
    ... )
    True
    """
    global TEST_CASES_CONFIG
    if not keys:
        raise ValueError("At least one key must be provided to load_test_cases_config")

    # Load the configuration file once and cache it for future accesses.
    if TEST_CASES_CONFIG is None:
        config_path = Path(__file__).parent / 'data' / 'test_cases_config.yaml'
        with open(config_path, 'r') as file:
            TEST_CASES_CONFIG = yaml.safe_load(file)

    config: Dict = TEST_CASES_CONFIG
    # Access nested dictionary using provided keys.
    for key in keys:
        config = config[key]

    # Extract values from the dict, to be a list of tuple
    return [tuple(test_case.values()) for test_case in config]


@pytest.fixture(scope="function")
def pdf_to_md_converter():
    return PdfToMdConverter()


@pytest.fixture(scope="function")
def md_to_md_converter():
    return MdToMdConverter()
