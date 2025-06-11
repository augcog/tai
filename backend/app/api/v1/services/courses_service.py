from typing import Tuple, List

from ..schemas.course import Course


def get_courses(page: int, limit: int, user: dict = None) -> Tuple[List[Course], int]:
    """
    Return a list of courses. If the user is authenticated, return both public and private courses;
    otherwise, return only public courses.
    """
    # TODO: Replace below mock data with actual database query
    if user:
        courses = [
            Course(courseId="CS61A", courseName="CS61A", isPublic=True),
            Course(courseId="CS61B", courseName="CS61B", isPublic=False),
        ]
    else:
        courses = [
            Course(courseId="CS61A", courseName="CS61A", isPublic=True),
        ]
    total = len(courses)
    # Simulate pagination
    start = (page - 1) * limit
    end = start + limit
    paged_courses = courses[start:end]
    return paged_courses, total
