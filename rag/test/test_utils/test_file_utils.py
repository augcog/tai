# file_conversion_router/tests/test_utils/test_file_utils.py
import pytest
from rag.file_conversion_router.utils.file_utils import setup_directory


def test_setup_directory(tmp_path):
    # Pytest automatically creates a tmp_path fixture that provides a temporary directory
    non_existing_path = tmp_path / "new_dir"
    assert not non_existing_path.exists(), "The directory should not exist yet"

    setup_directory(str(non_existing_path))

    assert non_existing_path.exists(), "The directory should have been created"
