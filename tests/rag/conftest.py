from pathlib import Path

import pytest
import yaml

# from rag.file_conversion_router.conversion.md_to_md import MdToMdConverter
from rag.file_conversion_router.conversion.pdf_to_md import PdfToMdConverter

# Define common base paths
BASE_PATH = Path(__file__).parent
DATA_FOLDER = BASE_PATH / "data"
CONFIG_FILE_PATH = DATA_FOLDER / "test_cases_config.yaml"
# Global variable to cache the test cases configuration
TEST_CASES_CONFIG = None


def load_yaml_config(config_path):
    """Load YAML configuration from a file."""
    with open(config_path, "r") as file:
        return yaml.safe_load(file)


def resolve_paths(config, base_path):
    """Recursively resolve paths in the configuration dictionary.

    Args:
        config (dict): The configuration dictionary.
        base_path (Path): The base path to resolve relative paths against.

    Modifies:
        config: Paths ending with '_path' or '_folder' are resolved to absolute paths.

    >>> test_config = {'input_path': 'input/example_input.pdf'}
    >>> resolve_paths(test_config, Path('/absolute/path/to/data'))
    >>> print(test_config)
    {'input_path': '/absolute/path/to/data/input/example_input.pdf'}
    """
    if isinstance(config, dict):
        for key, value in config.items():
            if key.endswith("_path") or key.endswith("_folder"):
                config[key] = str((base_path / value).resolve())
            else:
                resolve_paths(value, base_path)
    elif isinstance(config, list):
        for item in config:
            resolve_paths(item, base_path)


def load_test_cases_config(*keys):
    """Load and return configurations for specified test cases from a YAML file.

    This function loads and processes the configuration once and uses caching to store
    resolved paths. Subsequent calls use the cached data.

    Args:
        *keys (str): Variable length argument list of keys to access nested dictionary values.

    Returns:
        list: A list of tuples containing the values from the specified configuration entries.

    Raises:
        ValueError: If no keys are provided.

    >>> result = load_test_cases_config('unit_tests', 'example_format')
    >>> type(result) == list
    True
    >>> all(isinstance(item, tuple) for item in result)
    True
    >>> result[0][0].endswith('tests/rag/data/unit_tests/example_format/input/example_input.pdf')
    True
    >>> result[0][1].endswith('tests/rag/data/unit_tests/example_format/expected_output/example_output.mmd')
    True
    """
    global TEST_CASES_CONFIG
    if not keys:
        raise ValueError("At least one key must be provided")

    if TEST_CASES_CONFIG is None:
        raw_config = load_yaml_config(CONFIG_FILE_PATH)
        resolve_paths(raw_config, DATA_FOLDER)
        TEST_CASES_CONFIG = raw_config

    config = TEST_CASES_CONFIG
    for key in keys:
        config = config[key]

    return [tuple(test_case.values()) for test_case in config]


@pytest.fixture(scope="function")
def pdf_to_md_converter():
    return PdfToMdConverter()


# @pytest.fixture(scope="function")
# def md_to_md_converter():
#     return MdToMdConverter()
