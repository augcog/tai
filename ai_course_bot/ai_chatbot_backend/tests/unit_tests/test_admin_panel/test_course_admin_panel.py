import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from starlette.routing import Mount

from app.admin.admin import CourseAdmin, setup_admin
from app.core.models.courses import CourseModel


@pytest.fixture
def admin_app():
    """Create a FastAPI app with admin panel for testing."""
    app = FastAPI()
    # Just use the setup_admin function from the actual code
    setup_admin(app)
    return app


@pytest.fixture
def admin_panel_client(admin_app):
    """Create a test client for the admin panel."""
    return TestClient(admin_app)


class TestCourseAdminPanel:
    """Tests for the SQLAdmin admin panel integration."""

    def test_admin_initialization(self, admin_app):
        """Test that the admin panel is correctly initialized."""
        # Extract the admin mount from the app routes
        admin_mount = None
        for route in admin_app.routes:
            if isinstance(route, Mount) and route.path == "/admin":
                admin_mount = route
                break

        assert admin_mount is not None, "Admin panel mount not found in FastAPI app"
        assert admin_mount.path == "/admin", "Admin panel not mounted at /admin"

    def test_admin_panel_routes_accessible(self, admin_panel_client):
        """Test that admin panel routes are accessible."""
        # Main admin page
        response = admin_panel_client.get("/admin/")
        assert response.status_code == 200

        # Check for course admin in the HTML response
        assert "Course" in response.text

    def test_course_admin_model_attributes(self):
        """Test that CourseAdmin has the expected attributes and configurations."""
        course_admin = CourseAdmin()

        # Check that the model is correct
        assert course_admin.model == CourseModel

        # Check column configuration
        assert CourseModel.id in course_admin.column_list
        assert CourseModel.course_name in course_admin.column_list
        assert CourseModel.course_code in course_admin.column_list
        assert CourseModel.access_type in course_admin.column_list

        # Check form configuration
        assert "course_name" in course_admin.form_columns
        assert "course_code" in course_admin.form_columns
        assert "ip_address" in course_admin.form_columns
        assert "access_type" in course_admin.form_columns
        assert "school" in course_admin.form_columns

        # Check features
        assert course_admin.can_create
        assert course_admin.can_edit
        assert course_admin.can_delete
        assert course_admin.can_view_details

        # Check display name
        assert course_admin.name == "Course"
        assert course_admin.name_plural == "Courses"
