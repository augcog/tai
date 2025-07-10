"""
Tests for file metadata API endpoints
"""

from fastapi.testclient import TestClient
from sqlalchemy.orm import Session
from uuid import uuid4


class TestFileMetadataEndpoints:
    """Test file metadata API endpoints"""

    def test_create_file_metadata_success(self, client: TestClient, db_session: Session):
        """Test successful file metadata creation"""
        metadata_data = {
            "file_name": "lecture_notes.pdf",
            "url": "https://example.com/lecture_notes.pdf",
            "sections": [
                {
                    "name": "Introduction",
                    "index": 1.0,
                    "key_concept": "Variables",
                    "aspects": [
                        {"type": "summary", "content": "This section introduces variables."}
                    ]
                }
            ],
            "problems": [str(uuid4()), str(uuid4())]
        }
        
        response = client.post("/api/v1/file-metadata/", json=metadata_data)
        assert response.status_code == 201
        
        data = response.json()
        assert data["file_name"] == metadata_data["file_name"]
        assert data["url"] == metadata_data["url"]
        assert data["sections"] == metadata_data["sections"]
        assert data["problems"] == metadata_data["problems"]
        assert "uuid" in data
        assert "created_at" in data

    def test_create_file_metadata_duplicate_name(self, client: TestClient, db_session: Session):
        """Test file metadata creation with duplicate file name"""
        metadata_data = {
            "file_name": "lecture_notes.pdf",
            "url": "https://example.com/lecture_notes.pdf",
            "sections": [],
            "problems": []
        }
        
        # Create first metadata
        response = client.post("/api/v1/file-metadata/", json=metadata_data)
        assert response.status_code == 201
        
        # Try to create second metadata with same name
        metadata_data["url"] = "https://different.com/lecture_notes.pdf"
        response = client.post("/api/v1/file-metadata/", json=metadata_data)
        assert response.status_code == 400
        assert "File name already exists" in response.json()["detail"]

    def test_get_file_metadata_success(self, client: TestClient, db_session: Session):
        """Test successful file metadata retrieval"""
        # Create metadata first
        metadata_data = {
            "file_name": "lecture_notes.pdf",
            "url": "https://example.com/lecture_notes.pdf",
            "sections": [],
            "problems": []
        }
        create_response = client.post("/api/v1/file-metadata/", json=metadata_data)
        metadata_uuid = create_response.json()["uuid"]
        
        # Get the metadata
        response = client.get(f"/api/v1/file-metadata/{metadata_uuid}")
        assert response.status_code == 200
        
        data = response.json()
        assert data["file_name"] == metadata_data["file_name"]
        assert data["url"] == metadata_data["url"]

    def test_get_file_metadata_not_found(self, client: TestClient, db_session: Session):
        """Test getting non-existent file metadata"""
        response = client.get(f"/api/v1/file-metadata/{uuid4()}")
        assert response.status_code == 404
        assert "File metadata not found" in response.json()["detail"]

    def test_list_file_metadata(self, client: TestClient, db_session: Session):
        """Test file metadata listing"""
        # Create some metadata
        metadata_list = [
            {
                "file_name": "lecture_1.pdf",
                "url": "https://example.com/lecture_1.pdf",
                "sections": [],
                "problems": []
            },
            {
                "file_name": "lab_1.pdf",
                "url": "https://example.com/lab_1.pdf",
                "sections": [],
                "problems": []
            }
        ]
        
        for metadata_data in metadata_list:
            client.post("/api/v1/file-metadata/", json=metadata_data)
        
        # List metadata
        response = client.get("/api/v1/file-metadata/")
        assert response.status_code == 200
        
        data = response.json()
        assert data["total_count"] >= 2
        assert len(data["files"]) >= 2
        assert "page" in data
        assert "limit" in data
        assert "has_next" in data
        assert "has_prev" in data

    def test_update_file_metadata_success(self, client: TestClient, db_session: Session):
        """Test successful file metadata update"""
        # Create metadata first
        metadata_data = {
            "file_name": "lecture_notes.pdf",
            "url": "https://example.com/lecture_notes.pdf",
            "sections": [],
            "problems": []
        }
        create_response = client.post("/api/v1/file-metadata/", json=metadata_data)
        metadata_uuid = create_response.json()["uuid"]
        
        # Update the metadata
        update_data = {
            "url": "https://updated.com/lecture_notes.pdf",
            "sections": [
                {
                    "name": "Updated Section",
                    "index": 1.0,
                    "key_concept": "Updated Concept",
                    "aspects": [
                        {"type": "summary", "content": "Updated content."}
                    ]
                }
            ]
        }
        response = client.put(f"/api/v1/file-metadata/{metadata_uuid}", json=update_data)
        assert response.status_code == 200
        
        data = response.json()
        assert data["url"] == update_data["url"]
        assert data["sections"] == update_data["sections"]
        assert data["file_name"] == metadata_data["file_name"]  # Should remain unchanged

    def test_delete_file_metadata_success(self, client: TestClient, db_session: Session):
        """Test successful file metadata deletion"""
        # Create metadata first
        metadata_data = {
            "file_name": "lecture_notes.pdf",
            "url": "https://example.com/lecture_notes.pdf",
            "sections": [],
            "problems": []
        }
        create_response = client.post("/api/v1/file-metadata/", json=metadata_data)
        metadata_uuid = create_response.json()["uuid"]
        
        # Delete the metadata
        response = client.delete(f"/api/v1/file-metadata/{metadata_uuid}")
        assert response.status_code == 204
        
        # Verify metadata is soft deleted (not found)
        get_response = client.get(f"/api/v1/file-metadata/{metadata_uuid}")
        assert get_response.status_code == 404

    def test_add_problem_to_file(self, client: TestClient, db_session: Session):
        """Test adding a problem UUID to a file"""
        # Create metadata first
        metadata_data = {
            "file_name": "lecture_notes.pdf",
            "url": "https://example.com/lecture_notes.pdf",
            "sections": [],
            "problems": []
        }
        create_response = client.post("/api/v1/file-metadata/", json=metadata_data)
        metadata_uuid = create_response.json()["uuid"]
        
        # Add a problem
        problem_uuid = str(uuid4())
        response = client.post(f"/api/v1/file-metadata/{metadata_uuid}/problems/{problem_uuid}")
        assert response.status_code == 200
        
        # Verify problem was added
        get_response = client.get(f"/api/v1/file-metadata/{metadata_uuid}")
        assert get_response.status_code == 200
        assert problem_uuid in get_response.json()["problems"]

    def test_remove_problem_from_file(self, client: TestClient, db_session: Session):
        """Test removing a problem UUID from a file"""
        # Create metadata with a problem
        problem_uuid = str(uuid4())
        metadata_data = {
            "file_name": "lecture_notes.pdf",
            "url": "https://example.com/lecture_notes.pdf",
            "sections": [],
            "problems": [problem_uuid]
        }
        create_response = client.post("/api/v1/file-metadata/", json=metadata_data)
        metadata_uuid = create_response.json()["uuid"]
        
        # Remove the problem
        response = client.delete(f"/api/v1/file-metadata/{metadata_uuid}/problems/{problem_uuid}")
        assert response.status_code == 200
        
        # Verify problem was removed
        get_response = client.get(f"/api/v1/file-metadata/{metadata_uuid}")
        assert get_response.status_code == 200
        assert problem_uuid not in get_response.json()["problems"] 