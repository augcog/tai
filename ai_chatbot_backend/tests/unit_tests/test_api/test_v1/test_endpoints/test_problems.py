"""
Tests for problem API endpoints
"""

from fastapi.testclient import TestClient
from sqlalchemy.orm import Session
from uuid import uuid4


class TestProblemEndpoints:
    """Test problem API endpoints"""

    def test_create_problem_success(self, client: TestClient, practice_db_session: Session):
        """Test successful problem creation"""
        problem_data = {
            "question": "What is the correct value?",
            "choices": ["Option A", "Option B", "Option C"],
            "answer": 1,
            "explanation": "Option B is correct because..."
        }
        
        response = client.post("/api/v1/problems/", json=problem_data)
        assert response.status_code == 201
        
        data = response.json()
        assert data["question"] == problem_data["question"]
        assert data["choices"] == problem_data["choices"]
        assert data["answer"] == problem_data["answer"]
        assert data["explanation"] == problem_data["explanation"]
        assert "uuid" in data
        assert "created_at" in data

    def test_create_problem_invalid_answer(self, client: TestClient, practice_db_session: Session):
        """Test problem creation with invalid answer index"""
        problem_data = {
            "question": "What is the correct value?",
            "choices": ["Option A", "Option B"],
            "answer": 2,  # Invalid: should be 0 or 1
            "explanation": "Option B is correct because..."
        }
        
        response = client.post("/api/v1/problems/", json=problem_data)
        assert response.status_code == 400
        assert "Answer index must be less than the number of choices" in response.json()["detail"]

    def test_get_problem_success(self, client: TestClient, practice_db_session: Session):
        """Test successful problem retrieval"""
        # Create a problem first
        problem_data = {
            "question": "What is the correct value?",
            "choices": ["Option A", "Option B", "Option C"],
            "answer": 1,
            "explanation": "Option B is correct because..."
        }
        create_response = client.post("/api/v1/problems/", json=problem_data)
        problem_uuid = create_response.json()["uuid"]
        
        # Get the problem
        response = client.get(f"/api/v1/problems/{problem_uuid}")
        assert response.status_code == 200
        
        data = response.json()
        assert data["question"] == problem_data["question"]
        assert data["choices"] == problem_data["choices"]

    def test_get_problem_not_found(self, client: TestClient, practice_db_session: Session):
        """Test getting non-existent problem"""
        response = client.get(f"/api/v1/problems/{uuid4()}")
        assert response.status_code == 404
        assert "Problem not found" in response.json()["detail"]

    def test_list_problems(self, client: TestClient, practice_db_session: Session):
        """Test problem listing"""
        # Create some problems
        problems_data = [
            {
                "question": "Question 1?",
                "choices": ["A", "B"],
                "answer": 0,
                "explanation": "A is correct"
            },
            {
                "question": "Question 2?",
                "choices": ["X", "Y", "Z"],
                "answer": 1,
                "explanation": "Y is correct"
            }
        ]
        
        for problem_data in problems_data:
            client.post("/api/v1/problems/", json=problem_data)
        
        # List problems
        response = client.get("/api/v1/problems/")
        assert response.status_code == 200
        
        data = response.json()
        assert data["total_count"] >= 2
        assert len(data["problems"]) >= 2
        assert "page" in data
        assert "limit" in data
        assert "has_next" in data
        assert "has_prev" in data

    def test_update_problem_success(self, client: TestClient, practice_db_session: Session):
        """Test successful problem update"""
        # Create a problem first
        problem_data = {
            "question": "What is the correct value?",
            "choices": ["Option A", "Option B", "Option C"],
            "answer": 1,
            "explanation": "Option B is correct because..."
        }
        create_response = client.post("/api/v1/problems/", json=problem_data)
        problem_uuid = create_response.json()["uuid"]
        
        # Update the problem
        update_data = {
            "question": "Updated question?",
            "choices": ["New A", "New B"],
            "answer": 0,
            "explanation": "Updated explanation"
        }
        response = client.put(f"/api/v1/problems/{problem_uuid}", json=update_data)
        assert response.status_code == 200
        
        data = response.json()
        assert data["question"] == update_data["question"]
        assert data["choices"] == update_data["choices"]
        assert data["answer"] == update_data["answer"]
        assert data["explanation"] == update_data["explanation"]

    def test_delete_problem_success(self, client: TestClient, practice_db_session: Session):
        """Test successful problem deletion"""
        # Create a problem first
        problem_data = {
            "question": "What is the correct value?",
            "choices": ["Option A", "Option B"],
            "answer": 0,
            "explanation": "Option A is correct"
        }
        create_response = client.post("/api/v1/problems/", json=problem_data)
        problem_uuid = create_response.json()["uuid"]
        
        # Delete the problem
        response = client.delete(f"/api/v1/problems/{problem_uuid}")
        assert response.status_code == 204
        
        # Verify problem is soft deleted (not found)
        get_response = client.get(f"/api/v1/problems/{problem_uuid}")
        assert get_response.status_code == 404

    def test_validate_answer_correct(self, client: TestClient, practice_db_session: Session):
        """Test answer validation with correct answer"""
        # Create a problem first
        problem_data = {
            "question": "What is the correct value?",
            "choices": ["Option A", "Option B", "Option C"],
            "answer": 1,
            "explanation": "Option B is correct because..."
        }
        create_response = client.post("/api/v1/problems/", json=problem_data)
        problem_uuid = create_response.json()["uuid"]
        
        # Validate correct answer
        response = client.post(f"/api/v1/problems/{problem_uuid}/validate", json={"user_answer": 1})
        assert response.status_code == 200
        
        data = response.json()
        assert data["is_correct"] is True
        assert data["correct_answer"] == 1
        assert data["explanation"] == "Option B is correct because..."

    def test_validate_answer_incorrect(self, client: TestClient, practice_db_session: Session):
        """Test answer validation with incorrect answer"""
        # Create a problem first
        problem_data = {
            "question": "What is the correct value?",
            "choices": ["Option A", "Option B", "Option C"],
            "answer": 1,
            "explanation": "Option B is correct because..."
        }
        create_response = client.post("/api/v1/problems/", json=problem_data)
        problem_uuid = create_response.json()["uuid"]
        
        # Validate incorrect answer
        response = client.post(f"/api/v1/problems/{problem_uuid}/validate", json={"user_answer": 0})
        assert response.status_code == 200
        
        data = response.json()
        assert data["is_correct"] is False
        assert data["correct_answer"] == 1
        assert data["explanation"] == "Option B is correct because..."

    def test_get_problems_by_uuids(self, client: TestClient, practice_db_session: Session):
        """Test getting multiple problems by UUIDs"""
        # Create problems
        problems_data = [
            {
                "question": "Question 1?",
                "choices": ["A", "B"],
                "answer": 0,
                "explanation": "A is correct"
            },
            {
                "question": "Question 2?",
                "choices": ["X", "Y"],
                "answer": 1,
                "explanation": "Y is correct"
            }
        ]
        
        uuids = []
        for problem_data in problems_data:
            response = client.post("/api/v1/problems/", json=problem_data)
            uuids.append(response.json()["uuid"])
        
        # Get problems by UUIDs
        uuids_str = ",".join(uuids)
        response = client.get(f"/api/v1/problems/batch/{uuids_str}")
        assert response.status_code == 200
        
        data = response.json()
        assert len(data) == 2
        assert data[0]["question"] == "Question 1?"
        assert data[1]["question"] == "Question 2?" 