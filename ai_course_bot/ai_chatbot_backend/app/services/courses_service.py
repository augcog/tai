from typing import Tuple, List
from sqlalchemy.orm import Session

from app.schemas.course import Course
from app.core.models.courses import CourseModel


def get_courses(page: int, limit: int, user: dict = None, db: Session = None) -> Tuple[List[Course], int]:
    """
    Return a list of courses from the database. If the user is authenticated, return both public and private courses;
    otherwise, return only public courses.
    """
    if db is None:
        # Fallback mock data if no database session is provided
        if user:
            courses = [
                Course(courseId="mock-uuid-1",
                       courseName="CS61A", isPublic=True),
                Course(courseId="mock-uuid-2",
                       courseName="CS61B", isPublic=False),
            ]
        else:
            courses = [
                Course(courseId="mock-uuid-1",
                       courseName="CS61A", isPublic=True),
            ]
        total = len(courses)
        start = (page - 1) * limit
        end = start + limit
        paged_courses = courses[start:end]
        return paged_courses, total

    # Query database for courses
    query = db.query(CourseModel)

    # Filter by enabled courses only
    query = query.filter(CourseModel.enabled == True)

    # Filter by access type based on user authentication
    if user:
        # Authenticated users can see all enabled courses
        pass
    else:
        # Unauthenticated users can only see public courses
        query = query.filter(CourseModel.access_type == "public")

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
            isPublic=(db_course.access_type == "public")
        )
        courses.append(course)

    return courses, total
