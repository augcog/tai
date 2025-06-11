from fastapi import FastAPI
from sqladmin import Admin, ModelView

from app.core.database import engine
from app.core.models.courses import CourseModel


class CourseAdmin(ModelView, model=CourseModel):
    column_list = [
        CourseModel.id,
        CourseModel.course_name,
        CourseModel.course_code,
        CourseModel.ip_address,
        CourseModel.access_type,
        CourseModel.school
    ]
    column_details_exclude_list = []
    column_sortable_list = [CourseModel.id, CourseModel.course_name, CourseModel.course_code]
    column_searchable_list = [CourseModel.course_name, CourseModel.course_code]

    # Customize form for admin panel
    form_columns = [
        "course_name",
        "course_code",
        "ip_address",
        "access_type",
        "school"
    ]

    # Add validators and help text
    form_args = {
        "course_code": {
            "label": "Course Code",
            "description": "Unique identifier for the course (e.g., CS61A)"
        },
        "access_type": {
            "label": "Access Type",
            "description": "public - available to all, login_required - requires school login, private - restricted access"
        },
        "school": {
            "label": "School (if login required)",
            "description": "Only required when Access Type is set to 'login_required'"
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
    admin = Admin(app, engine)

    # Register ModelView
    admin.add_view(CourseAdmin)
