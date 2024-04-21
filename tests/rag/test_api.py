# file_conversion_router/tests/test_api.py
from unittest.mock import patch

from rag.file_conversion_router import api


def test_convert_directory():
    with patch("file_conversion_router.services.directory_service.process_folder") as mocked_process:
        api.convert_directory("input", "output")
        mocked_process.assert_called_once_with("input", "output")
