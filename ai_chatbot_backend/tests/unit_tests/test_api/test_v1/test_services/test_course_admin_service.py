import pytest
from typing import List, Tuple, Optional

from app.api.v1.schemas.course_admin import CourseCreate, CourseUpdate
from app.api.v1.services import course_admin_service
from app.core.models.courses import CourseModel

from tests.common.test_base_classes.course_admin_base import BaseCourseAdminTest


class TestCourseAdminService(BaseCourseAdminTest):
    """
    Tests for the course admin service functions.
    Inherits common test cases from BaseCourseAdminTest.
    """

    @pytest.fixture(autouse=True)
    def setup(self, course_db_session):
        """Setup for each test - store db session."""
        self.db = course_db_session

    def create_course(self, course_data: CourseCreate) -> CourseModel:
        """Implementation of create_course for service tests."""
        return course_admin_service.create_course(self.db, course_data)

    def get_course(self, course_id: int) -> Optional[CourseModel]:
        """Implementation of get_course for service tests."""
        return course_admin_service.get_course(self.db, course_id)

    def get_course_by_code(self, course_code: str) -> Optional[CourseModel]:
        """Implementation of get_course_by_code for service tests."""
        return course_admin_service.get_course_by_code(self.db, course_code)

    def update_course(
        self, course_id: int, course_data: CourseUpdate
    ) -> Optional[CourseModel]:
        """Implementation of update_course for service tests."""
        return course_admin_service.update_course(self.db, course_id, course_data)

    def delete_course(self, course_id: int) -> bool:
        """Implementation of delete_course for service tests."""
        return course_admin_service.delete_course(self.db, course_id)

    def get_courses(
        self, skip: int = 0, limit: int = 100
    ) -> Tuple[List[CourseModel], int]:
        """Implementation of get_courses for service tests."""
        return course_admin_service.get_courses(self.db, skip, limit)

    # Additional service-specific tests

    def test_create_duplicate_course_code(self, sample_course_data, course_fixture):
        """Test that creating a course with an existing course code fails gracefully."""
        # Create a new course data object with the same course code
        duplicate_data = CourseCreate(
            course_name="Another Course",
            course_code=course_fixture.course_code,  # Same code as existing course
            ip_address="127.0.0.2",
            access_type=sample_course_data.access_type,
        )

        # The service itself doesn't enforce uniqueness, so we'd expect this to
        # either raise an exception or return the course based on the implementation
        # For this test, we'll check that SQLAlchemy would raise an IntegrityError
        # which we'd handle at the endpoint level
        with pytest.raises(Exception) as excinfo:
            course_admin_service.create_course(self.db, duplicate_data)
            self.db.flush()  # This would trigger the database constraint

        # Assert that an integrity or uniqueness error was raised
        assert "UNIQUE constraint failed" in str(
            excinfo.value
        ) or "IntegrityError" in str(excinfo.value)

    def test_update_course_code_to_existing(
        self, course_fixture, login_required_course_fixture
    ):
        """Test that updating a course with an existing course code fails gracefully."""
        # Try to update course to have same code as login_required_course
        update_data = CourseUpdate(
            course_code=login_required_course_fixture.course_code
        )

        # Again, the service itself doesn't enforce uniqueness, so we'd expect an error on flush
        with pytest.raises(Exception) as excinfo:
            course_admin_service.update_course(self.db, course_fixture.id, update_data)
            self.db.flush()  # This would trigger the database constraint

        # Assert that an integrity or uniqueness error was raised
        assert "UNIQUE constraint failed" in str(
            excinfo.value
        ) or "IntegrityError" in str(excinfo.value)
