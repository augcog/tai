from typing import List, Optional
from pydantic import BaseModel, Field
from enum import Enum


class AccessType(str, Enum):
    PUBLIC = "public"
    LOGIN_REQUIRED = "login_required"
    PRIVATE = "private"


class CourseBase(BaseModel):
    course_name: str
    server_url: str
    enabled: bool = True
    access_type: AccessType = AccessType.PUBLIC
    school: Optional[str] = None


class CourseCreate(CourseBase):
    pass


class CourseUpdate(BaseModel):
    course_name: Optional[str] = None
    server_url: Optional[str] = None
    enabled: Optional[bool] = None
    access_type: Optional[AccessType] = None
    school: Optional[str] = None


class CourseResponse(CourseBase):
    id: int
    course_id: str

    class Config:
        orm_mode = True
        from_attributes = True


class CourseListResponse(BaseModel):
    courses: List[CourseResponse]
    total: int
