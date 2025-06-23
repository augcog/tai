from typing import List

from pydantic import BaseModel


class Course(BaseModel):
    courseId: str
    courseName: str
    semester: str
    isPublic: bool
    order: int
    school: str
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
