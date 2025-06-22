from fastapi import FastAPI
from sqladmin import Admin, ModelView

from app.core.database import engine
from app.core.models.courses import CourseModel
from app.admin.admin_auth import AdminAuth
from app.config import settings


class CourseAdmin(ModelView, model=CourseModel):
    column_list = [
        CourseModel.id,
        CourseModel.course_id,
        CourseModel.course_name,
        CourseModel.server_url,
        CourseModel.enabled,
        CourseModel.access_type,
        CourseModel.school,
        CourseModel.order
    ]
    column_details_exclude_list = []
    column_sortable_list = [CourseModel.id,
                            CourseModel.course_name, CourseModel.enabled, CourseModel.order]
    column_searchable_list = [CourseModel.course_name,
                              CourseModel.server_url, CourseModel.school]

    # Customize form for admin panel
    form_columns = [
        "course_name",
        "server_url",
        "enabled",
        "access_type",
        "school",
        "order"
    ]

    # Add validators and help text
    form_args = {
        "course_name": {
            "label": "Course Name",
            "description": "The full name of the course (e.g., Introduction to Computer Science)"
        },
        "server_url": {
            "label": "Server URL",
            "description": "The server URL for this course"
        },
        "enabled": {
            "label": "Enabled",
            "description": "Whether this course is enabled and available"
        },
        "access_type": {
            "label": "Access Type",
            "description": "public - available to all, login_required - requires school login, private - restricted access"
        },
        "school": {
            "label": "School (if login required)",
            "description": "Only required when Access Type is set to 'login_required'"
        },
        "order": {
            "label": "Order",
            "description": "The order of the course in the list"
        }
    }

    can_create = True
    can_edit = True
    can_delete = True
    can_view_details = True
    name = "Course"
    name_plural = "Courses"
    icon = "fa-solid fa-graduation-cap"


def setup_admin(app: FastAPI):
    admin = Admin(app, engine, authentication_backend=AdminAuth(
        secret_key=settings.admin_token))

    # Register ModelView
    admin.add_view(CourseAdmin)
