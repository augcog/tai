import os
import tempfile
from unittest.mock import patch, MagicMock

import pytest
from fastapi import HTTPException
from fastapi.testclient import TestClient
from fastapi.responses import FileResponse

from app.api.deps import get_current_user
from app.api.v1.services.file_storage import local_storage
from main import app

client = TestClient(app)


@pytest.fixture
def mock_authorization_header():
    """Create a mock authorization header"""
    return {"Authorization": "Bearer test_token"}


@pytest.fixture
def mock_current_user():
    """Override authentication during testing"""
    def _mock_get_current_user():
        return {"sub": "test-user"}
    
    app.dependency_overrides[get_current_user] = _mock_get_current_user
    yield
    app.dependency_overrides.clear()


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
def mock_pdf_file():
    """Create a mock PDF LocalFile instance"""
    return local_storage.LocalFile(
        file_name="test.pdf",
        file_path="documents/test.pdf",
        mime_type="application/pdf",
        size_bytes=500,
        modified_time="2023-01-01T00:00:00",
        directory="documents"
    )


@pytest.fixture
def mock_video_file():
    """Create a mock video LocalFile instance"""
    return local_storage.LocalFile(
        file_name="lecture.mp4",
        file_path="videos/lecture.mp4",
        mime_type="video/mp4",
        size_bytes=10000,
        modified_time="2023-01-01T00:00:00",
        directory="videos"
    )


class TestListFiles:
    """Unit tests for the list_files endpoint"""

    def test_list_files(self, mock_authorization_header, mock_file, mock_current_user):
        """Test basic file listing"""
        with patch.object(local_storage, 'list_files') as mock_list_files:
            mock_list_files.return_value = [mock_file]
            
            response = client.get("/v1/local-files", headers=mock_authorization_header)
            
            assert response.status_code == 200
            data = response.json()
            assert "files" in data
            assert len(data["files"]) == 1
            assert data["total_count"] == 1

    def test_list_files_with_directory(self, mock_authorization_header, mock_file, mock_current_user):
        """Test file listing with directory parameter"""
        with patch.object(local_storage, 'list_files') as mock_list_files:
            mock_list_files.return_value = [mock_file]
            
            response = client.get(
                "/v1/local-files?directory=documents",
                headers=mock_authorization_header
            )
            
            assert response.status_code == 200
            mock_list_files.assert_called_once_with(directory="documents")

    def test_list_files_with_multiple_types(self, mock_authorization_header, mock_file, 
                                           mock_pdf_file, mock_video_file, mock_current_user):
        """Test listing files of multiple types"""
        with patch.object(local_storage, 'list_files') as mock_list_files:
            mock_list_files.return_value = [mock_file, mock_pdf_file, mock_video_file]
            
            response = client.get("/v1/local-files", headers=mock_authorization_header)
            
            assert response.status_code == 200
            data = response.json()
            assert len(data["files"]) == 3
            assert data["total_count"] == 3
            
            # Verify different file types are included
            file_types = {file["mime_type"] for file in data["files"]}
            assert "text/plain" in file_types
            assert "application/pdf" in file_types
            assert "video/mp4" in file_types

    def test_list_empty_directory(self, mock_authorization_header, mock_current_user):
        """Test listing an empty directory"""
        with patch.object(local_storage, 'list_files') as mock_list_files:
            mock_list_files.return_value = []
            
            response = client.get("/v1/local-files", headers=mock_authorization_header)
            
            assert response.status_code == 200
            data = response.json()
            assert data["files"] == []
            assert data["total_count"] == 0


class TestGetFile:
    """Unit tests for the get_file endpoint"""

    def test_get_text_file(self, mock_authorization_header, mock_current_user):
        """Test retrieving a text file"""
        # Create a temporary text file
        with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as temp_file:
            test_content = b"Test content for text file"
            temp_file.write(test_content)
            temp_file_path = temp_file.name
        
        try:
            with patch.object(local_storage, 'get_file') as mock_get_file:
                mock_file_response = FileResponse(
                    path=temp_file_path,
                    media_type="text/plain",
                    filename=os.path.basename(temp_file_path)
                )
                mock_get_file.return_value = mock_file_response
                
                response = client.get(
                    "/v1/local-files/documents/test.txt",
                    headers=mock_authorization_header
                )
                
                assert response.status_code == 200
                assert response.content == test_content
                assert response.headers["content-type"].startswith("text/plain")
        finally:
            os.unlink(temp_file_path)

    def test_get_pdf_file(self, mock_authorization_header, mock_current_user):
        """Test retrieving a PDF file"""
        # Create a temporary PDF file
        with tempfile.NamedTemporaryFile(suffix=".pdf", delete=False) as temp_file:
            test_content = b"%PDF-1.5\nTest PDF content\n%%EOF"
            temp_file.write(test_content)
            temp_file_path = temp_file.name
        
        try:
            with patch.object(local_storage, 'get_file') as mock_get_file:
                mock_file_response = FileResponse(
                    path=temp_file_path,
                    media_type="application/pdf",
                    filename=os.path.basename(temp_file_path)
                )
                mock_get_file.return_value = mock_file_response
                
                response = client.get(
                    "/v1/local-files/documents/test.pdf",
                    headers=mock_authorization_header
                )
                
                assert response.status_code == 200
                assert response.content == test_content
                assert response.headers["content-type"].startswith("application/pdf")
        finally:
            os.unlink(temp_file_path)

    def test_get_video_file(self, mock_authorization_header, mock_current_user):
        """Test retrieving a video file"""
        # Create a temporary video file
        with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as temp_file:
            test_content = b"Mock video content"
            temp_file.write(test_content)
            temp_file_path = temp_file.name
        
        try:
            with patch.object(local_storage, 'get_file') as mock_get_file:
                mock_file_response = FileResponse(
                    path=temp_file_path,
                    media_type="video/mp4",
                    filename=os.path.basename(temp_file_path)
                )
                mock_get_file.return_value = mock_file_response
                
                response = client.get(
                    "/v1/local-files/videos/lecture.mp4",
                    headers=mock_authorization_header
                )
                
                assert response.status_code == 200
                assert response.content == test_content
                assert response.headers["content-type"].startswith("video/mp4")
        finally:
            os.unlink(temp_file_path)
    
    def test_get_file_not_found(self, mock_authorization_header, mock_current_user):
        """Test getting a non-existent file"""
        with patch.object(local_storage, 'get_file') as mock_get_file:
            mock_get_file.side_effect = HTTPException(status_code=404, detail="File not found")
            
            response = client.get(
                "/v1/local-files/documents/nonexistent.txt",
                headers=mock_authorization_header
            )
            
            assert response.status_code == 404
            assert "detail" in response.json()

    def test_security_path_traversal(self, mock_authorization_header, mock_current_user):
        """Test security against path traversal attempts"""
        with patch.object(local_storage, 'get_file') as mock_get_file:
            mock_get_file.side_effect = HTTPException(
                status_code=403,
                detail="Access denied: Path is outside the allowed directory"
            )
            
            # Note: The URL handling in FastAPI test client works differently than real requests
            response = client.get(
                "/v1/local-files/../../../../etc/passwd",
                headers=mock_authorization_header
            )
            
            # Either 403 or 404 is acceptable as long as access is denied
            assert response.status_code in (403, 404)
            if response.status_code == 403:
                assert "Access denied" in response.json()["detail"]
            else:
                assert "detail" in response.json() 