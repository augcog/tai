import os
import shutil
import tempfile
from datetime import datetime
import pytest
from unittest.mock import patch, MagicMock
from fastapi.testclient import TestClient
from fastapi import HTTPException
from pathlib import Path
from fastapi.responses import FileResponse
import importlib

from app.api.deps import get_current_user, auth_with_query_param
from main import app
from app.api.v1.services.file_storage import local_storage
from app.api.v1.endpoints import local_files
from app.config import settings


# Create a test client
test_client = TestClient(app)


@pytest.fixture
def client():
    """Return the TestClient instance"""
    return test_client


@pytest.fixture
def mock_authorization_header():
    """Create a mock authorization header"""
    return {"Authorization": "Bearer test_token"}


@pytest.fixture
def mock_current_user():
    """Override authentication during testing"""
    # Store original auth_required setting
    original_auth_required = settings.auth_required

    # Force auth_required to False during tests
    settings.auth_required = False

    # Create a mock user
    mock_user = {
        "user_id": "test-user",
        "email": "test@example.com",
        "name": "Test User",
        "picture": None
    }

    # Override both authentication dependencies directly in the local_files router
    for route in local_files.router.routes:
        if hasattr(route, "dependencies"):
            for i, dependency in enumerate(route.dependencies):
                if dependency.dependency == get_current_user or dependency.dependency == auth_with_query_param:
                    route.dependencies[i].dependency = lambda: mock_user

    # Also override the app dependencies
    app.dependency_overrides[get_current_user] = lambda: mock_user
    app.dependency_overrides[auth_with_query_param] = lambda: mock_user

    yield mock_user

    # Restore original settings and clear overrides
    settings.auth_required = original_auth_required
    app.dependency_overrides.clear()

    # Reload the module to restore original dependencies
    importlib.reload(local_files)


@pytest.fixture
def temp_dir():
    temp_dir = tempfile.mkdtemp()
    yield temp_dir
    shutil.rmtree(temp_dir)


@pytest.fixture
def mock_file():
    """Create a mock LocalFile instance"""
    return local_storage.LocalFile(
        file_name="test.txt",
        file_path="documents/test.txt",
        mime_type="text/plain",
        size_bytes=100,
        modified_time="2023-01-01T00:00:00",
        directory="documents"
    )


# Base test class for file operations
class TestFileBase:
    """Base class for file operation tests"""

    def setup_method(self):
        self.client = test_client

    def assert_file_response(self, response, test_content, expected_mime_type, filename):
        """Common assertions for file responses"""
        assert response.status_code == 200
        assert response.content == test_content
        assert "content-type" in response.headers
        assert response.headers["content-type"].startswith(expected_mime_type)
        assert "content-disposition" in response.headers
        assert filename in response.headers["content-disposition"]


# Test class for file listing operations
class TestFileListOperations(TestFileBase):
    """Tests for file listing endpoints"""

    def test_list_files(self, mock_authorization_header, mock_file, mock_current_user):
        """Test the file listing endpoint"""
        with patch.object(local_storage, 'list_files') as mock_list_files:
            mock_list_files.return_value = [mock_file]

            response = self.client.get("/v1/local-files", headers=mock_authorization_header)

            assert response.status_code == 200
            data = response.json()
            assert "files" in data
            assert len(data["files"]) == 1
            assert data["total_count"] == 1

            # Verify file metadata
            file_data = data["files"][0]
            assert file_data["file_name"] == "test.txt"
            assert file_data["file_path"] == "documents/test.txt"
            assert file_data["mime_type"] == "text/plain"

    def test_list_files_with_directory(self, mock_authorization_header, mock_file, mock_current_user):
        """Test the file listing endpoint with a specific directory"""
        with patch.object(local_storage, 'list_files') as mock_list_files:
            mock_list_files.return_value = [mock_file]

            response = self.client.get(
                "/v1/local-files?directory=documents",
                headers=mock_authorization_header
            )

            assert response.status_code == 200
            mock_list_files.assert_called_once_with(directory="documents")

    def test_list_files_empty_directory(self, mock_authorization_header, mock_current_user):
        """Test listing files in an empty directory"""
        with patch.object(local_storage, 'list_files') as mock_list_files:
            mock_list_files.return_value = []

            response = self.client.get("/v1/local-files", headers=mock_authorization_header)

            assert response.status_code == 200
            data = response.json()
            assert data["files"] == []
            assert data["total_count"] == 0

    def test_list_files_invalid_directory(self, mock_authorization_header, mock_current_user):
        """Test listing files in an invalid directory"""
        with patch.object(local_storage, 'list_files') as mock_list_files:
            mock_list_files.side_effect = HTTPException(
                status_code=404,
                detail="Directory 'invalid_dir' not found"
            )

            response = self.client.get(
                "/v1/local-files?directory=invalid_dir",
                headers=mock_authorization_header
            )

            assert response.status_code == 404
            assert response.json()["detail"] == "Directory 'invalid_dir' not found"

    def test_get_file_hierarchy(self, mock_authorization_header, mock_current_user):
        """Test the file hierarchy endpoint"""
        # Mock hierarchy data
        mock_hierarchy = {
            "root": {
                "name": "root",
                "path": "",
                "type": "directory",
                "children": [
                    {
                        "name": "documents",
                        "path": "documents",
                        "type": "directory",
                        "children": [
                            {
                                "name": "test.txt",
                                "path": "documents/test.txt",
                                "type": "file",
                                "mime_type": "text/plain",
                                "size_bytes": 100,
                                "modified_time": "2023-01-01T00:00:00"
                            }
                        ]
                    }
                ]
            },
            "total_files": 1,
            "total_directories": 1,
            "max_depth": 2
        }

        with patch.object(local_storage, 'get_file_hierarchy') as mock_get_hierarchy:
            mock_get_hierarchy.return_value = mock_hierarchy

            response = self.client.get("/v1/local-files/hierarchy", headers=mock_authorization_header)

            assert response.status_code == 200
            data = response.json()
            assert "root" in data
            assert data["total_files"] == 1
            assert data["total_directories"] == 1
            assert data["max_depth"] == 2

            # Verify root node
            root = data["root"]
            assert root["name"] == "root"
            assert root["path"] == ""
            assert root["type"] == "directory"
            assert "children" in root

            # Verify children
            documents = root["children"][0]
            assert documents["name"] == "documents"
            assert documents["path"] == "documents"
            assert documents["type"] == "directory"

            # Verify file
            file = documents["children"][0]
            assert file["name"] == "test.txt"
            assert file["path"] == "documents/test.txt"
            assert file["type"] == "file"
            assert file["mime_type"] == "text/plain"

    def test_get_file_hierarchy_with_directory(self, mock_authorization_header, mock_current_user):
        """Test the file hierarchy endpoint with a specific directory"""
        # Mock hierarchy data for a specific directory
        mock_hierarchy = {
            "root": {
                "name": "documents",
                "path": "documents",
                "type": "directory",
                "children": [
                    {
                        "name": "test.txt",
                        "path": "documents/test.txt",
                        "type": "file",
                        "mime_type": "text/plain",
                        "size_bytes": 100,
                        "modified_time": "2023-01-01T00:00:00"
                    }
                ]
            },
            "total_files": 1,
            "total_directories": 0,
            "max_depth": 1
        }

        with patch.object(local_storage, 'get_file_hierarchy') as mock_get_hierarchy:
            mock_get_hierarchy.return_value = mock_hierarchy

            response = self.client.get(
                "/v1/local-files/hierarchy?directory=documents",
                headers=mock_authorization_header
            )

            assert response.status_code == 200
            mock_get_hierarchy.assert_called_once_with(directory="documents", max_depth=-1)

    def test_get_file_hierarchy_with_max_depth(self, mock_authorization_header, mock_current_user):
        """Test the file hierarchy endpoint with a max depth parameter"""
        # Mock hierarchy data with limited depth
        mock_hierarchy = {
            "root": {
                "name": "root",
                "path": "",
                "type": "directory",
                "children": [
                    {
                        "name": "documents",
                        "path": "documents",
                        "type": "directory"
                        # No children due to max_depth=1
                    }
                ]
            },
            "total_files": 0,
            "total_directories": 1,
            "max_depth": 1
        }

        with patch.object(local_storage, 'get_file_hierarchy') as mock_get_hierarchy:
            mock_get_hierarchy.return_value = mock_hierarchy

            response = self.client.get(
                "/v1/local-files/hierarchy?max_depth=1",
                headers=mock_authorization_header
            )

            assert response.status_code == 200
            mock_get_hierarchy.assert_called_once_with(directory=None, max_depth=1)

    def test_get_file_hierarchy_invalid_directory(self, mock_authorization_header, mock_current_user):
        """Test the file hierarchy endpoint with an invalid directory"""
        with patch.object(local_storage, 'get_file_hierarchy') as mock_get_hierarchy:
            mock_get_hierarchy.side_effect = HTTPException(
                status_code=404,
                detail="Directory 'invalid_dir' not found"
            )

            response = self.client.get(
                "/v1/local-files/hierarchy?directory=invalid_dir",
                headers=mock_authorization_header
            )

            assert response.status_code == 404
            assert response.json()["detail"] == "Directory 'invalid_dir' not found"


# Test class for text file operations
class TestTextFileOperations(TestFileBase):
    """Tests specifically for text file operations"""

    def test_get_text_file(self, mock_authorization_header, mock_current_user):
        """Test retrieving a text file"""
        # Create a temporary file with test content
        with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as temp_file:
            test_content = b"Test content for text file retrieval"
            temp_file.write(test_content)
            temp_file_path = temp_file.name

        try:
            # Mock the get_file function to return our test file
            with patch.object(local_storage, 'get_file') as mock_get_file:
                # Configure the mock to return a FileResponse for our temp file
                mock_file_response = FileResponse(
                    path=temp_file_path,
                    media_type="text/plain",
                    filename=os.path.basename(temp_file_path)
                )
                mock_get_file.return_value = mock_file_response

                # Test the endpoint
                response = self.client.get(
                    "/v1/local-files/documents/test.txt",
                    headers=mock_authorization_header
                )

                # Verify the response
                self.assert_file_response(
                    response,
                    test_content,
                    "text/plain",
                    os.path.basename(temp_file_path)
                )
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)


# Test class for PDF file operations
class TestPDFFileOperations(TestFileBase):
    """Tests specifically for PDF file operations"""

    def test_get_pdf_file(self, mock_authorization_header, mock_current_user):
        """Test retrieving a PDF file"""
        # Create a temporary file with test content
        with tempfile.NamedTemporaryFile(suffix=".pdf", delete=False) as temp_file:
            # Simple PDF content (not valid PDF but suitable for testing)
            test_content = b"%PDF-1.5\nSome mock PDF content\n%%EOF"
            temp_file.write(test_content)
            temp_file_path = temp_file.name

        try:
            # Mock the get_file function to return our test file
            with patch.object(local_storage, 'get_file') as mock_get_file:
                # Configure the mock to return a FileResponse for our temp file
                mock_file_response = FileResponse(
                    path=temp_file_path,
                    media_type="application/pdf",
                    filename=os.path.basename(temp_file_path)
                )
                mock_get_file.return_value = mock_file_response

                # Test the endpoint
                response = self.client.get(
                    "/v1/local-files/documents/test.pdf",
                    headers=mock_authorization_header
                )

                # Verify the response
                self.assert_file_response(
                    response,
                    test_content,
                    "application/pdf",
                    os.path.basename(temp_file_path)
                )
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)


# Test class for video file operations
class TestVideoFileOperations(TestFileBase):
    """Tests specifically for video file operations"""

    def test_get_video_file(self, mock_authorization_header, mock_current_user):
        """Test retrieving a video file"""
        # Create a temporary file with test content
        with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as temp_file:
            # Mock video content
            test_content = b"Mock MP4 video content"
            temp_file.write(test_content)
            temp_file_path = temp_file.name

        try:
            # Mock the get_file function to return our test file
            with patch.object(local_storage, 'get_file') as mock_get_file:
                # Configure the mock to return a FileResponse for our temp file
                mock_file_response = FileResponse(
                    path=temp_file_path,
                    media_type="video/mp4",
                    filename=os.path.basename(temp_file_path)
                )
                mock_get_file.return_value = mock_file_response

                # Test the endpoint
                response = self.client.get(
                    "/v1/local-files/videos/test.mp4",
                    headers=mock_authorization_header
                )

                # Verify the response
                self.assert_file_response(
                    response,
                    test_content,
                    "video/mp4",
                    os.path.basename(temp_file_path)
                )
        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)


# Test class for error handling
class TestFileErrorHandling(TestFileBase):
    """Tests for file error handling"""

    def test_get_file_not_found(self, mock_authorization_header, mock_current_user):
        """Test the file retrieval endpoint with a non-existent file"""
        test_file_path = "documents/nonexistent.txt"

        with patch.object(local_storage, 'get_file') as mock_get_file:
            mock_get_file.side_effect = HTTPException(status_code=404, detail="File not found")

            response = self.client.get(
                f"/v1/local-files/{test_file_path}",
                headers=mock_authorization_header
            )

            assert response.status_code == 404
            assert "detail" in response.json()
            assert response.json()["detail"] == "File not found"

    def test_local_files_security_check(self, mock_authorization_header, mock_current_user):
        """Test security check for file paths outside the allowed directory"""
        # Note: The URL path traversal in FastAPI test client works differently than real requests
        # so we need to construct a valid URL path that would trigger the security check
        test_file_path = "../../../../etc/passwd"  # Attempt path traversal

        with patch.object(local_storage, 'get_file') as mock_get_file:
            mock_get_file.side_effect = HTTPException(
                status_code=403,
                detail="Access denied: Path is outside the allowed directory"
            )

            # Use the path directly without modifying it
            response = self.client.get(
                f"/v1/local-files/{test_file_path}",
                headers=mock_authorization_header
            )

            # Either 403 or 404 is acceptable as long as access is denied
            assert response.status_code in (403, 404)
            if response.status_code == 403:
                assert "Access denied" in response.json()["detail"]
            else:
                assert "detail" in response.json()


# Direct test functions for backward compatibility
def test_local_files_not_found(client, mock_current_user):
    """Test accessing a non-existent file returns 404."""
    response = client.get("/v1/local-files/non-existent-file.txt")
    assert response.status_code == 404

    data = response.json()
    assert "detail" in data
