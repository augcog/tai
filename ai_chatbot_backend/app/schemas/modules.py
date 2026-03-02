"""
Module schemas for API responses
"""

from typing import List, Optional
from pydantic import BaseModel, Field


class Module(BaseModel):
    """Represents a module (subdirectory) within a course"""

    name: str = Field(..., description="Module name (directory name)")
    path: str = Field(..., description="Full path relative to course root")
    category: str = Field(..., description="Module category (practice, study, support)")
    file_count: int = Field(..., description="Number of files in this module")
    course_code: str = Field(..., description="Course code this module belongs to")

    class Config:
        json_schema_extra = {
            "example": {
                "name": "week1",
                "path": "study/week1",
                "category": "study",
                "file_count": 5,
                "course_code": "CS61A"
            }
        }


class ModuleListResponse(BaseModel):
    """Response for module listing"""

    modules: List[Module] = Field(..., description="List of modules")
    total_count: int = Field(..., description="Total number of modules")
    course_code: str = Field(..., description="Course code")

    class Config:
        json_schema_extra = {
            "example": {
                "modules": [
                    {
                        "name": "week1",
                        "path": "study/week1",
                        "category": "study",
                        "file_count": 5,
                        "course_code": "CS61A"
                    },
                    {
                        "name": "lab01",
                        "path": "practice/labs/lab01",
                        "category": "practice",
                        "file_count": 3,
                        "course_code": "CS61A"
                    }
                ],
                "total_count": 2,
                "course_code": "CS61A"
            }
        }
