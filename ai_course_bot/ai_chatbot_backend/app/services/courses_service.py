from typing import Tuple, List
from sqlalchemy.orm import Session

from app.schemas.course import Course
from app.core.models.courses import CourseModel


def get_courses(
    page: int, limit: int, user: dict = None, db: Session = None
) -> Tuple[List[Course], int]:
    """
    Return a list of courses from the database. If the user is authenticated, return both public and private courses;
    otherwise, return only public courses.
    """
    # Query database for courses
    query = db.query(CourseModel)

    # Filter by enabled courses only
    query = query.filter(CourseModel.enabled == True)

    # Get total count before pagination
    total = query.count()

    # Apply pagination
    offset = (page - 1) * limit
    db_courses = query.offset(offset).limit(limit).all()

    # Convert to response format
    courses = []
    for db_course in db_courses:
        course = Course(
            courseId=db_course.course_id,
            courseName=db_course.course_name,
            accessType=db_course.access_type,
            order=db_course.order,
            school=db_course.school,
            serverUrl=db_course.server_url,
            courseCode=db_course.course_code,
            semester=db_course.semester,
        )
        courses.append(course)

    return courses, total
