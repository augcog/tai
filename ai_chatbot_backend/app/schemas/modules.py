"""
Module schemas for API responses
"""

from typing import List
from pydantic import BaseModel, Field


class Module(BaseModel):
    """Represents a module (subdirectory) within a course"""

    module_uuid: str = Field(..., description="Unique module identifier")
    name: str = Field(..., description="Module name (directory name)")
    path: str = Field(..., description="Full path relative to course root")
    category: str = Field(..., description="Module category (practice, study, support)")
    course_code: str = Field(..., description="Course code this module belongs to")

    class Config:
        json_schema_extra = {
            "example": {
                "module_uuid": "550e8400-e29b-41d4-a716-446655440000",
                "name": "week1",
                "path": "study/week1",
                "category": "study",
                "course_code": "CS61A"
            }
        }

    @classmethod
    def from_db_model(cls, m) -> "Module":
        return cls(
            module_uuid=m.uuid,
            name=m.name,
            path=m.path,
            category=m.category,
            course_code=m.course_code,
        )


class ModuleListResponse(BaseModel):
    """Response for module listing"""

    modules: List[Module] = Field(..., description="List of modules")
    total_count: int = Field(..., description="Total number of modules")
    course_code: str = Field(..., description="Course code")
