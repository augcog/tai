"""
File metadata schemas for API requests and responses
"""

from typing import Optional, List
from pydantic import BaseModel, Field, validator
from uuid import UUID

class Aspect(BaseModel):
    type: str
    content: str

class Section(BaseModel):
    name: str
    index: float
    key_concept: str
    aspects: List[Aspect]

class FileMetadataBase(BaseModel):
    file_name: str = Field(..., min_length=1, max_length=255, description="File name (unique)")
    url: Optional[str] = Field(None, description="File URL")
    sections: Optional[List[Section]] = Field(None, description="Sections of the file")

class FileMetadataCreate(FileMetadataBase):
    pass

class FileMetadataUpdate(BaseModel):
    file_name: Optional[str] = Field(None, min_length=1, max_length=255)
    url: Optional[str] = Field(None)
    sections: Optional[List[Section]] = Field(None)

class FileMetadataResponse(FileMetadataBase):
    uuid: UUID

    @validator('sections', pre=True)
    def convert_sections_to_models(cls, v):
        """Convert JSON dictionaries to Section models"""
        if v is None:
            return v
        if isinstance(v, list):
            return [Section(**section) if isinstance(section, dict) else section for section in v]
        return v

    class Config:
        from_attributes = True
        json_schema_extra = {
            "example": {
                "uuid": "550e8400-e29b-41d4-a716-446655440000",
                "file_name": "lecture_notes.pdf",
                "url": "https://example.com/lecture_notes.pdf",
                "sections": [
                    {
                        "name": "Introduction",
                        "index": 1.0,
                        "key_concept": "Variables",
                        "aspects": [
                            {"type": "summary", "content": "This section introduces variables."}
                        ]
                    }
                ]
            }
        }

class FileMetadataListResponse(BaseModel):
    files: List[FileMetadataResponse]
    total_count: int
    page: int
    limit: int
    has_next: bool
    has_prev: bool

class FileMetadataListParams(BaseModel):
    search: Optional[str] = Field(None, description="Search in file name")
    page: int = Field(1, ge=1, description="Page number")
    limit: int = Field(100, ge=1, le=1000, description="Items per page")

class ProblemDetail(BaseModel):
    """Problem detail for response"""
    uuid: str = Field(..., description="Problem UUID")
    file_uuid: str = Field(..., description="File UUID")
    problem_index: Optional[str] = Field(None, description="Problem index")
    problem_id: Optional[str] = Field(None, description="Problem ID")
    problem_content: Optional[str] = Field(None, description="Problem content")
    question_id: Optional[int] = Field(None, description="Question ID")
    question: str = Field(..., description="Question text")
    choices: List[str] = Field(..., description="List of choices")
    answer: int = Field(..., description="Correct answer index")
    explanation: Optional[str] = Field(None, description="Answer explanation")

class ProblemsByFileNameListResponse(BaseModel):
    """Response for problems by file name query"""
    file_name: str = Field(..., description="File name")
    problems: List[ProblemDetail] = Field(..., description="List of problems for this file")
    
    class Config:
        json_schema_extra = {
            "example": {
                "file_name": "test_file.py",
                "problems": [
                    {
                        "uuid": "550e8400-e29b-41d4-a716-446655440001",
                        "file_uuid": "550e8400-e29b-41d4-a716-446655440000",
                        "problem_index": "1.1",
                        "problem_id": "PROB001",
                        "question": "What is the capital of France?",
                        "choices": ["Berlin", "Paris", "London"],
                        "answer": 1,
                        "explanation": "Paris is the capital of France."
                    },
                    {
                        "uuid": "550e8400-e29b-41d4-a716-446655440002",
                        "file_uuid": "550e8400-e29b-41d4-a716-446655440000",
                        "problem_index": "1.2",
                        "problem_id": "PROB002",
                        "question": "What is 2 + 2?",
                        "choices": ["3", "4", "5"],
                        "answer": 1,
                        "explanation": "2 + 2 = 4."
                    }
                ]
            }
        }
