from sqlalchemy import Column, Integer, String, Enum
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()


class CourseModel(Base):
    __tablename__ = "courses"

    id = Column(Integer, primary_key=True, index=True)
    course_name = Column(String, index=True)
    course_code = Column(String, unique=True, index=True)
    ip_address = Column(String)
    access_type = Column(
        Enum("public", "login_required", "private", name="access_type_enum"),
        default="public"
    )
    school = Column(String, nullable=True)  # Only used when access_type is login_required

    def __repr__(self):
        return f"<Course(id={self.id}, name={self.course_name}, code={self.course_code})>"
