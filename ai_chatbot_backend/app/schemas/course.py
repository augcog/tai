from typing import List, Optional

from pydantic import BaseModel


class Course(BaseModel):
    courseId: str
    courseName: str
    semester: Optional[str] = None
    accessType: str
    order: int
    school: Optional[str] = None
    serverUrl: str
    courseCode: str


class Meta(BaseModel):
    page: int
    limit: int
    total: int
    totalPages: int


class CoursesResponse(BaseModel):
    data: List[Course]
    meta: Meta
