from abc import ABC, abstractmethod

from app.api.v1.schemas.course_admin import CourseCreate, CourseUpdate, AccessType


class BaseCourseAdminTest(ABC):
    """
    Base test class for course admin tests.
    This provides common test cases that can be inherited by both service and endpoint tests.
    """

    @abstractmethod
    def create_course(self, course_data: CourseCreate):
        """Method to create a course, implemented by service or endpoint tests."""
        pass

    @abstractmethod
    def get_course(self, course_id: int):
        """Method to get a course by ID, implemented by service or endpoint tests."""
        pass

    @abstractmethod
    def get_course_by_code(self, course_code: str):
        """Method to get a course by code, implemented by service or endpoint tests."""
        pass

    @abstractmethod
    def update_course(self, course_id: int, course_data: CourseUpdate):
        """Method to update a course, implemented by service or endpoint tests."""
        pass

    @abstractmethod
    def delete_course(self, course_id: int):
        """Method to delete a course, implemented by service or endpoint tests."""
        pass

    @abstractmethod
    def get_courses(self, skip: int = 0, limit: int = 100):
        """Method to get a list of courses, implemented by service or endpoint tests."""
        pass

    def test_create_course(self, sample_course_data):
        """Test creating a new course."""
        course = self.create_course(sample_course_data)
        assert course is not None
        assert course.course_name == sample_course_data.course_name
        assert course.course_code == sample_course_data.course_code
        assert course.ip_address == sample_course_data.ip_address
        assert course.access_type == sample_course_data.access_type

    def test_create_login_required_course(self, sample_login_required_course_data):
        """Test creating a course with login_required access type."""
        course = self.create_course(sample_login_required_course_data)
        assert course is not None
        assert course.access_type == AccessType.LOGIN_REQUIRED
        assert course.school == sample_login_required_course_data.school

    def test_get_course(self, course_fixture):
        """Test retrieving a course by ID."""
        course = self.get_course(course_fixture.id)
        assert course is not None
        assert course.id == course_fixture.id
        assert course.course_name == course_fixture.course_name
        assert course.course_code == course_fixture.course_code

    def test_get_nonexistent_course(self):
        """Test retrieving a nonexistent course."""
        course = self.get_course(999)
        assert course is None

    def test_get_course_by_code(self, course_fixture):
        """Test retrieving a course by course code."""
        course = self.get_course_by_code(course_fixture.course_code)
        assert course is not None
        assert course.id == course_fixture.id
        assert course.course_name == course_fixture.course_name

    def test_get_nonexistent_course_by_code(self):
        """Test retrieving a nonexistent course by code."""
        course = self.get_course_by_code("NONEXISTENT")
        assert course is None

    def test_update_course(self, course_fixture):
        """Test updating a course."""
        update_data = CourseUpdate(
            course_name="Updated Course Name",
            ip_address="192.168.1.1"
        )
        updated_course = self.update_course(course_fixture.id, update_data)
        assert updated_course is not None
        assert updated_course.id == course_fixture.id
        assert updated_course.course_name == "Updated Course Name"
        assert updated_course.ip_address == "192.168.1.1"
        assert updated_course.course_code == course_fixture.course_code  # Unchanged field

    def test_update_course_access_type(self, course_fixture):
        """Test updating a course's access type."""
        # Change to login_required and add school
        update_data = CourseUpdate(
            access_type=AccessType.LOGIN_REQUIRED,
            school="Test School"
        )
        updated_course = self.update_course(course_fixture.id, update_data)
        assert updated_course is not None
        assert updated_course.access_type == AccessType.LOGIN_REQUIRED
        assert updated_course.school == "Test School"

        # Change to private (school should be set to None)
        update_data = CourseUpdate(
            access_type=AccessType.PRIVATE
        )
        updated_course = self.update_course(course_fixture.id, update_data)
        assert updated_course is not None
        assert updated_course.access_type == AccessType.PRIVATE
        assert updated_course.school is None

    def test_update_nonexistent_course(self):
        """Test updating a nonexistent course."""
        update_data = CourseUpdate(
            course_name="Updated Course Name"
        )
        updated_course = self.update_course(999, update_data)
        assert updated_course is None

    def test_delete_course(self, course_fixture):
        """Test deleting a course."""
        result = self.delete_course(course_fixture.id)
        assert result is True

        # Verify the course is gone
        course = self.get_course(course_fixture.id)
        assert course is None

    def test_delete_nonexistent_course(self):
        """Test deleting a nonexistent course."""
        result = self.delete_course(999)
        assert result is False

    def test_get_courses_pagination(self, multiple_courses_fixture):
        """Test retrieving courses with pagination."""
        # Get first page (limit 10)
        courses, total = self.get_courses(skip=0, limit=10)
        assert len(courses) <= 10
        assert total == len(multiple_courses_fixture)

        # Get second page (limit 10)
        courses_page_2, _ = self.get_courses(skip=10, limit=10)

        # Verify different courses on different pages
        if len(courses_page_2) > 0 and len(courses) > 0:
            assert courses[0].id != courses_page_2[0].id
