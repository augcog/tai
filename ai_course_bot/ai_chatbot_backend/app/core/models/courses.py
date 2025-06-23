from sqlalchemy import Column, Integer, String, Enum, Boolean
from app.core.database import Base
import uuid


class CourseModel(Base):
    __tablename__ = "courses"

    id = Column(Integer, primary_key=True, index=True)
    course_id = Column(String, unique=True, index=True,
                       default=lambda: str(uuid.uuid4()))
    course_name = Column(String, index=True)
    server_url = Column(String, nullable=False)
    course_code = Column(String, nullable=False)
    semester = Column(String, nullable=True)
    enabled = Column(Boolean, default=True)
    order = Column(Integer, default=0)
    access_type = Column(
        Enum("public", "login_required", "private", name="access_type_enum"),
        default="public"
    )
    # Only used when access_type is login_required
    school = Column(String, nullable=True)

    def __repr__(self):
        return f"<Course(id={self.id}, name={self.course_name}, course_id={self.course_id})>"
