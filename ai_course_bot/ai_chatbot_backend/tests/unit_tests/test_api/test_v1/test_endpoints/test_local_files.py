import os
import tempfile
from unittest.mock import patch

import pytest
from fastapi import HTTPException
from fastapi.testclient import TestClient
from fastapi.responses import FileResponse

from app.api.deps import get_current_user
from app.api.v1.services.file_storage import local_storage
# Use relative imports for app modules in the unit test directory
from main import app

client = TestClient(app)


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


@pytest.fixture
def mock_authorization_header():
    """Create a mock authorization header"""
    return {"Authorization": "Bearer test_token"}


def test_list_files(mock_authorization_header, mock_file):
    """Test the file listing endpoint"""
    with patch.object(local_storage, 'list_files') as mock_list_files:
        mock_list_files.return_value = [mock_file]

        # Override authentication during testing
        def mock_get_current_user():
            return {"sub": "test-user"}

        app.dependency_overrides[get_current_user] = mock_get_current_user

        response = client.get("/v1/local-files", headers=mock_authorization_header)

        # Clean up dependencies
        app.dependency_overrides.clear()

        assert response.status_code == 200
        data = response.json()
        assert "files" in data
        assert len(data["files"]) == 1
        assert data["total_count"] == 1


def test_list_files_with_directory(mock_authorization_header, mock_file):
    """Test the file listing endpoint with a specific directory"""
    with patch.object(local_storage, 'list_files') as mock_list_files:
        mock_list_files.return_value = [mock_file]

        # Override authentication during testing
        def mock_get_current_user():
            return {"sub": "test-user"}

        app.dependency_overrides[get_current_user] = mock_get_current_user

        response = client.get(
            "/v1/local-files?directory=documents",
            headers=mock_authorization_header
        )

        # Clean up dependencies
        app.dependency_overrides.clear()

        assert response.status_code == 200
        mock_list_files.assert_called_once_with(directory="documents")


def test_get_file(mock_authorization_header):
    """Test the file retrieval endpoint"""
    # Create a temporary file with test content
    with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as temp_file:
        test_content = b"Test content for file retrieval"
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
            
            # Override authentication during testing
            def mock_get_current_user():
                return {"sub": "test-user"}

            app.dependency_overrides[get_current_user] = mock_get_current_user
            
            # Test the endpoint
            response = client.get(
                "/v1/local-files/documents/test.txt",
                headers=mock_authorization_header
            )
            
            # Clean up dependencies
            app.dependency_overrides.clear()
            
            # Verify the response
            assert response.status_code == 200
            assert response.content == test_content
            assert "content-type" in response.headers
            assert response.headers["content-type"].startswith("text/plain")
            assert "content-disposition" in response.headers
            assert os.path.basename(temp_file_path) in response.headers["content-disposition"]
    finally:
        # Clean up the temporary file
        os.unlink(temp_file_path)


def test_get_file_not_found(mock_authorization_header):
    """Test the file retrieval endpoint with a non-existent file"""
    test_file_path = "documents/nonexistent.txt"

    with patch.object(local_storage, 'get_file') as mock_get_file:
        mock_get_file.side_effect = HTTPException(status_code=404, detail="File not found")

        # Override authentication during testing
        def mock_get_current_user():
            return {"sub": "test-user"}

        app.dependency_overrides[get_current_user] = mock_get_current_user

        response = client.get(
            f"/v1/local-files/{test_file_path}",
            headers=mock_authorization_header
        )

        # Clean up dependencies
        app.dependency_overrides.clear()

        assert response.status_code == 404
