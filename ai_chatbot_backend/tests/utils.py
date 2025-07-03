"""
Test utilities for the AI Course Bot Backend tests
"""

import os
import tempfile
import shutil
from typing import Dict, Any, List, Optional

from fastapi.responses import FileResponse

from app.api.v1.services.file_storage import local_storage


class TestFileManager:
    """Utility class for managing test files"""

    def __init__(self):
        self.temp_files: List[str] = []
        self.temp_dirs: List[str] = []

    def create_temp_file(self, content: bytes, suffix: str = ".txt") -> str:
        """Create a temporary file with given content"""
        temp_file = tempfile.NamedTemporaryFile(suffix=suffix, delete=False)
        temp_file.write(content)
        temp_file.close()

        self.temp_files.append(temp_file.name)
        return temp_file.name

    def create_temp_dir(self) -> str:
        """Create a temporary directory"""
        temp_dir = tempfile.mkdtemp()
        self.temp_dirs.append(temp_dir)
        return temp_dir

    def cleanup(self):
        """Clean up all temporary files and directories"""
        for file_path in self.temp_files:
            try:
                os.unlink(file_path)
            except (OSError, FileNotFoundError):
                pass

        for dir_path in self.temp_dirs:
            try:
                shutil.rmtree(dir_path)
            except (OSError, FileNotFoundError):
                pass

        self.temp_files.clear()
        self.temp_dirs.clear()


class MockFileBuilder:
    """Builder class for creating mock file objects"""

    def __init__(self):
        self.reset()

    def reset(self):
        """Reset to default values"""
        self._file_name = "test.txt"
        self._file_path = "documents/test.txt"
        self._mime_type = "text/plain"
        self._size_bytes = 100
        self._modified_time = "2023-01-01T00:00:00"
        self._directory = "documents"
        return self

    def with_name(self, name: str):
        """Set the file name"""
        self._file_name = name
        return self

    def with_path(self, path: str):
        """Set the file path"""
        self._file_path = path
        return self

    def with_mime_type(self, mime_type: str):
        """Set the MIME type"""
        self._mime_type = mime_type
        return self

    def with_size(self, size: int):
        """Set the file size"""
        self._size_bytes = size
        return self

    def with_directory(self, directory: str):
        """Set the directory"""
        self._directory = directory
        return self

    def build(self) -> local_storage.LocalFile:
        """Build the mock file object"""
        return local_storage.LocalFile(
            file_name=self._file_name,
            file_path=self._file_path,
            mime_type=self._mime_type,
            size_bytes=self._size_bytes,
            modified_time=self._modified_time,
            directory=self._directory,
        )


class APITestHelper:
    """Helper class for API testing"""

    @staticmethod
    def assert_error_response(
        response, expected_status: int, expected_detail: Optional[str] = None
    ):
        """Assert that a response is an error with expected status and detail"""
        assert response.status_code == expected_status, (
            f"Expected status {expected_status}, got {response.status_code}: {response.text}"
        )

        data = response.json()
        assert "detail" in data, "Error response should contain 'detail' field"

        if expected_detail:
            assert expected_detail in data["detail"], (
                f"Expected detail to contain '{expected_detail}', got '{data['detail']}'"
            )

    @staticmethod
    def assert_file_list_response(response, expected_count: Optional[int] = None):
        """Assert that a response is a valid file list response"""
        assert response.status_code == 200, (
            f"Expected 200, got {response.status_code}: {response.text}"
        )

        data = response.json()
        assert "files" in data, "Response should contain 'files' field"
        assert "total_count" in data, "Response should contain 'total_count' field"
        assert isinstance(data["files"], list), "'files' should be a list"
        assert isinstance(data["total_count"], int), (
            "'total_count' should be an integer"
        )

        if expected_count is not None:
            assert len(data["files"]) == expected_count, (
                f"Expected {expected_count} files, got {len(data['files'])}"
            )
            assert data["total_count"] == expected_count, (
                f"Expected total_count {expected_count}, got {data['total_count']}"
            )

        # Validate file structure
        for file_data in data["files"]:
            assert "file_name" in file_data, "File should have 'file_name'"
            assert "file_path" in file_data, "File should have 'file_path'"
            assert "mime_type" in file_data, "File should have 'mime_type'"
            assert "size_bytes" in file_data, "File should have 'size_bytes'"

    @staticmethod
    def assert_hierarchy_response(response):
        """Assert that a response is a valid hierarchy response"""
        assert response.status_code == 200, (
            f"Expected 200, got {response.status_code}: {response.text}"
        )

        data = response.json()
        assert "root" in data, "Response should contain 'root' field"
        assert "total_files" in data, "Response should contain 'total_files' field"
        assert "total_directories" in data, (
            "Response should contain 'total_directories' field"
        )
        assert "max_depth" in data, "Response should contain 'max_depth' field"

        # Validate root structure
        root = data["root"]
        assert "name" in root, "Root should have 'name'"
        assert "path" in root, "Root should have 'path'"
        assert "type" in root, "Root should have 'type'"
        assert root["type"] == "directory", "Root should be a directory"

    @staticmethod
    def create_mock_file_response(
        file_path: str, mime_type: str, filename: str
    ) -> FileResponse:
        """Create a mock FileResponse for testing"""
        return FileResponse(path=file_path, media_type=mime_type, filename=filename)


class TestDataFactory:
    """Factory for creating test data"""

    @staticmethod
    def create_file_hierarchy(depth: int = 2, files_per_dir: int = 2) -> Dict[str, Any]:
        """Create a mock file hierarchy for testing"""

        def create_directory(
            name: str, path: str, current_depth: int
        ) -> Dict[str, Any]:
            children = []

            # Add files
            for i in range(files_per_dir):
                file_name = f"file_{i}.txt"
                file_path = f"{path}/{file_name}" if path else file_name
                children.append(
                    {
                        "name": file_name,
                        "path": file_path,
                        "type": "file",
                        "mime_type": "text/plain",
                        "size_bytes": 100 + i,
                        "modified_time": "2023-01-01T00:00:00",
                    }
                )

            # Add subdirectories if we haven't reached max depth
            if current_depth < depth:
                for i in range(2):  # 2 subdirectories per level
                    subdir_name = f"subdir_{i}"
                    subdir_path = f"{path}/{subdir_name}" if path else subdir_name
                    children.append(
                        create_directory(subdir_name, subdir_path, current_depth + 1)
                    )

            return {
                "name": name,
                "path": path,
                "type": "directory",
                "children": children,
            }

        root = create_directory("root", "", 0)
        total_files = files_per_dir * (2**depth)  # Approximate
        total_directories = (2**depth) - 1  # Approximate

        return {
            "root": root,
            "total_files": total_files,
            "total_directories": total_directories,
            "max_depth": depth,
        }

    @staticmethod
    def create_file_list(
        count: int, directory: str = "documents"
    ) -> List[local_storage.LocalFile]:
        """Create a list of mock files for testing"""
        files = []
        for i in range(count):
            files.append(
                MockFileBuilder()
                .with_name(f"test_file_{i}.txt")
                .with_path(f"{directory}/test_file_{i}.txt")
                .with_size(100 + i)
                .with_directory(directory)
                .build()
            )
        return files


# Test constants
TEST_FILE_CONTENT = {
    "text": b"This is test content for a text file",
    "pdf": b"%PDF-1.5\nMock PDF content\n%%EOF",
    "video": b"Mock MP4 video content",
    "image": b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde",
}

TEST_MIME_TYPES = {
    "text": "text/plain",
    "pdf": "application/pdf",
    "video": "video/mp4",
    "image": "image/png",
}

# Common test file extensions
TEST_FILE_EXTENSIONS = {
    "text": [".txt", ".md", ".py", ".js"],
    "pdf": [".pdf"],
    "video": [".mp4", ".avi", ".mov"],
    "image": [".png", ".jpg", ".jpeg", ".gif"],
}
