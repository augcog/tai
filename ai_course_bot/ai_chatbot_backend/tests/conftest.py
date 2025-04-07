import os
import pytest
from fastapi.testclient import TestClient
from fastapi import HTTPException, status
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from app.core.models.courses import Base as CourseBase
from app.api.v1.schemas.course_admin import CourseCreate, AccessType
from app.api.deps import get_current_user, get_admin_user
from app.core.database import get_db
from app.api.v1.services import course_admin_service
from main import app

# Create a test database engine
SQLALCHEMY_DATABASE_URL = "sqlite:///:memory:"
engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


# ===== User Fixtures =====

def dummy_admin_user():
    """Dummy admin user for testing."""
    return {
        "user_id": "admin_user",
        "email": "admin@example.com",
        "name": "Admin User",
        "is_admin": True
    }


def dummy_regular_user():
    """Dummy regular user for testing."""
    return {
        "user_id": "regular_user",
        "email": "user@example.com",
        "name": "Regular User",
        "is_admin": False
    }


# ===== Database Fixtures =====

@pytest.fixture
def course_db_session():
    """Create a fresh database session for testing courses."""
    CourseBase.metadata.create_all(bind=engine)
    session = TestingSessionLocal()
    try:
        yield session
    finally:
        session.close()
        CourseBase.metadata.drop_all(bind=engine)


# ===== Client Fixtures =====

@pytest.fixture
def admin_client(course_db_session):
    """Test client with admin user permissions."""
    def override_get_db():
        try:
            yield course_db_session
        finally:
            pass

    app.dependency_overrides[get_db] = override_get_db
    app.dependency_overrides[get_current_user] = dummy_admin_user
    app.dependency_overrides[get_admin_user] = dummy_admin_user
    
    with TestClient(app) as client:
        yield client
    
    # Clear overrides after test
    app.dependency_overrides.clear()


def reject_non_admin_user():
    """Reject non-admin users with a 403 Forbidden error."""
    raise HTTPException(
        status_code=status.HTTP_403_FORBIDDEN,
        detail="Admin access required for this endpoint"
    )


@pytest.fixture
def regular_client(course_db_session):
    """Test client with regular user permissions."""
    def override_get_db():
        try:
            yield course_db_session
        finally:
            pass

    app.dependency_overrides[get_db] = override_get_db
    app.dependency_overrides[get_current_user] = dummy_regular_user
    app.dependency_overrides[get_admin_user] = reject_non_admin_user
    
    with TestClient(app) as client:
        yield client
    
    # Clear overrides after test
    app.dependency_overrides.clear()


# ===== Course Admin Fixtures =====

@pytest.fixture
def sample_course_data():
    """Provide sample course data for tests."""
    return CourseCreate(
        course_name="Test Course",
        course_code="TEST101",
        ip_address="127.0.0.1",
        access_type=AccessType.PUBLIC,
        school=None
    )


@pytest.fixture
def sample_login_required_course_data():
    """Provide sample course data with login_required for tests."""
    return CourseCreate(
        course_name="School Course",
        course_code="SCHOOL101",
        ip_address="127.0.0.1",
        access_type=AccessType.LOGIN_REQUIRED,
        school="Test University"
    )


@pytest.fixture
def sample_private_course_data():
    """Provide sample private course data for tests."""
    return CourseCreate(
        course_name="Private Course",
        course_code="PRIV101",
        ip_address="127.0.0.1",
        access_type=AccessType.PRIVATE,
        school=None
    )


@pytest.fixture
def multiple_course_data():
    """Provide multiple course data entries for pagination tests."""
    return [
        CourseCreate(
            course_name=f"Test Course {i}",
            course_code=f"TEST{i:03d}",
            ip_address="127.0.0.1",
            access_type=AccessType.PUBLIC,
            school=None
        )
        for i in range(1, 21)  # Creates 20 test courses
    ]


@pytest.fixture
def course_fixture(course_db_session, sample_course_data):
    """Create a course in the database for testing."""
    course = course_admin_service.create_course(course_db_session, sample_course_data)
    return course


@pytest.fixture
def login_required_course_fixture(course_db_session, sample_login_required_course_data):
    """Create a login_required course in the database for testing."""
    course = course_admin_service.create_course(course_db_session, sample_login_required_course_data)
    return course


@pytest.fixture
def private_course_fixture(course_db_session, sample_private_course_data):
    """Create a private course in the database for testing."""
    course = course_admin_service.create_course(course_db_session, sample_private_course_data)
    return course


@pytest.fixture
def multiple_courses_fixture(course_db_session, multiple_course_data):
    """Create multiple courses in the database for testing pagination."""
    courses = []
    for course_data in multiple_course_data:
        course = course_admin_service.create_course(course_db_session, course_data)
        courses.append(course)
    return courses 