from typing import List, Optional, Tuple

from sqlalchemy.orm import Session

from app.api.v1.schemas.course_admin import CourseCreate, CourseUpdate
from app.core.models.courses import CourseModel


def create_course(db: Session, course: CourseCreate) -> CourseModel:
    """Create a new course in the database."""
    db_course = CourseModel(
        course_name=course.course_name,
        course_code=course.course_code,
        ip_address=course.ip_address,
        access_type=course.access_type,
        school=course.school if course.access_type == "login_required" else None
    )
    db.add(db_course)
    db.commit()
    db.refresh(db_course)
    return db_course


def get_courses(db: Session, skip: int = 0, limit: int = 100) -> Tuple[List[CourseModel], int]:
    """Get all courses with pagination."""
    courses = db.query(CourseModel).offset(skip).limit(limit).all()
    total = db.query(CourseModel).count()
    return courses, total


def get_course(db: Session, course_id: int) -> Optional[CourseModel]:
    """Get a course by ID."""
    return db.query(CourseModel).filter(CourseModel.id == course_id).first()


def get_course_by_code(db: Session, course_code: str) -> Optional[CourseModel]:
    """Get a course by course code."""
    return db.query(CourseModel).filter(CourseModel.course_code == course_code).first()


def update_course(db: Session, course_id: int, course_update: CourseUpdate) -> Optional[CourseModel]:
    """Update a course by ID."""
    db_course = get_course(db, course_id)
    if db_course is None:
        return None

    update_data = course_update.dict(exclude_unset=True)

    # Special handling for access_type and school
    if "access_type" in update_data and update_data["access_type"] != "login_required":
        update_data["school"] = None

    for key, value in update_data.items():
        setattr(db_course, key, value)

    db.commit()
    db.refresh(db_course)
    return db_course


def delete_course(db: Session, course_id: int) -> bool:
    """Delete a course by ID."""
    db_course = get_course(db, course_id)
    if db_course is None:
        return False

    db.delete(db_course)
    db.commit()
    return True
