import pytest
from typing import List, Tuple, Optional
from fastapi import status

from app.api.v1.schemas.course_admin import CourseCreate, CourseUpdate, CourseResponse
from app.core.models.courses import CourseModel

from tests.common.test_base_classes.course_admin_base import BaseCourseAdminTest


class TestCourseAdminEndpoints(BaseCourseAdminTest):
    """
    Tests for the course admin API endpoints.
    Inherits common test cases from BaseCourseAdminTest.
    """

    @pytest.fixture(autouse=True)
    def setup(self, admin_client):
        """Setup for each test - store client."""
        self.client = admin_client
        self.base_url = "/v1/course-admin"

    def create_course(self, course_data: CourseCreate) -> CourseModel:
        """Implementation of create_course for endpoint tests."""
        response = self.client.post(self.base_url + "/", json=course_data.dict())
        assert response.status_code == status.HTTP_201_CREATED
        return CourseModel(**response.json())

    def get_course(self, course_id: int) -> Optional[CourseModel]:
        """Implementation of get_course for endpoint tests."""
        response = self.client.get(f"{self.base_url}/{course_id}")
        if response.status_code == status.HTTP_404_NOT_FOUND:
            return None
        assert response.status_code == status.HTTP_200_OK
        return CourseModel(**response.json())

    def get_course_by_code(self, course_code: str) -> Optional[CourseModel]:
        """
        Implementation of get_course_by_code for endpoint tests.

        Note: If the endpoint doesn't directly support getting by code,
        we can implement this by getting all courses and filtering.
        """
        # Assuming the endpoint doesn't have a direct "get by code" method
        # so we'll get all courses and filter
        response = self.client.get(f"{self.base_url}/?limit=100")
        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        courses = [CourseModel(**course) for course in data.get("courses", [])]

        for course in courses:
            if course.course_code == course_code:
                return course

        return None

    def update_course(
        self, course_id: int, course_data: CourseUpdate
    ) -> Optional[CourseModel]:
        """Implementation of update_course for endpoint tests."""
        response = self.client.put(
            f"{self.base_url}/{course_id}",
            json={k: v for k, v in course_data.dict().items() if v is not None},
        )
        if response.status_code == status.HTTP_404_NOT_FOUND:
            return None
        assert response.status_code == status.HTTP_200_OK
        return CourseModel(**response.json())

    def delete_course(self, course_id: int) -> bool:
        """Implementation of delete_course for endpoint tests."""
        response = self.client.delete(f"{self.base_url}/{course_id}")
        if response.status_code == status.HTTP_404_NOT_FOUND:
            return False
        return response.status_code == status.HTTP_204_NO_CONTENT

    def get_courses(
        self, skip: int = 0, limit: int = 100
    ) -> Tuple[List[CourseModel], int]:
        """Implementation of get_courses for endpoint tests."""
        response = self.client.get(f"{self.base_url}/?skip={skip}&limit={limit}")
        assert response.status_code == status.HTTP_200_OK

        data = response.json()
        courses = [CourseModel(**course) for course in data.get("courses", [])]
        total = data.get("total", 0)

        return courses, total

    # Additional endpoint-specific tests

    def test_create_course_returns_correct_status(self, sample_course_data):
        """Test that creating a course returns 201 Created status code."""
        response = self.client.post(self.base_url + "/", json=sample_course_data.dict())
        assert response.status_code == status.HTTP_201_CREATED

    def test_create_duplicate_course_code_returns_400(
        self, sample_course_data, course_fixture
    ):
        """Test that creating a course with an existing code returns 400 Bad Request."""
        # Create new course data with the same code
        duplicate_data = CourseCreate(
            course_name="Another Course",
            course_code=course_fixture.course_code,  # Same code as existing course
            ip_address="127.0.0.2",
            access_type=sample_course_data.access_type,
        )

        response = self.client.post(self.base_url + "/", json=duplicate_data.dict())
        assert response.status_code == status.HTTP_400_BAD_REQUEST
        assert "already exists" in response.json().get("detail", "")

    def test_get_nonexistent_course_returns_404(self):
        """Test that getting a nonexistent course returns 404 Not Found."""
        response = self.client.get(f"{self.base_url}/999")
        assert response.status_code == status.HTTP_404_NOT_FOUND

    def test_update_nonexistent_course_returns_404(self):
        """Test that updating a nonexistent course returns 404 Not Found."""
        update_data = CourseUpdate(course_name="New Name")
        response = self.client.put(
            f"{self.base_url}/999", json=update_data.dict(exclude_unset=True)
        )
        assert response.status_code == status.HTTP_404_NOT_FOUND

    def test_update_course_with_existing_code_returns_400(
        self, course_fixture, login_required_course_fixture
    ):
        """Test that updating a course with an existing code returns 400 Bad Request."""
        update_data = CourseUpdate(
            course_code=login_required_course_fixture.course_code
        )

        response = self.client.put(
            f"{self.base_url}/{course_fixture.id}",
            json=update_data.dict(exclude_unset=True),
        )
        assert response.status_code == status.HTTP_400_BAD_REQUEST
        assert "already exists" in response.json().get("detail", "")

    def test_delete_nonexistent_course_returns_404(self):
        """Test that deleting a nonexistent course returns 404 Not Found."""
        response = self.client.delete(f"{self.base_url}/999")
        assert response.status_code == status.HTTP_404_NOT_FOUND

    def test_regular_user_cannot_access_admin_endpoints(
        self, regular_client, sample_course_data
    ):
        """Test that regular users cannot access admin endpoints."""
        # Try to create a course
        response = regular_client.post(
            "/v1/course-admin/",
            json=sample_course_data.dict(),
            headers={"Authorization": "Bearer regular_user_token"},
        )
        # Should return 403 Forbidden (or possibly 401 Unauthorized depending on implementation)
        assert response.status_code in [
            status.HTTP_401_UNAUTHORIZED,
            status.HTTP_403_FORBIDDEN,
        ]
