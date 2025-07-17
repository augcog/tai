"""
Problem schemas for API requests and responses
"""

from typing import List, Optional, Union
from pydantic import BaseModel, Field
from uuid import UUID
from datetime import datetime

class ProblemBase(BaseModel):
    """Base problem schema with common fields"""
    file_uuid: UUID = Field(..., description="File UUID this problem belongs to")
    problem_index: Optional[str] = Field(None, description="Problem index within the file")
    problem_id: Optional[str] = Field(None, description="Problem identifier")
    problem_content: Optional[str] = Field(None, description="Problem content")
    question_id: Optional[int] = Field(None, description="Question ID")
    question: str = Field(..., description="The question text")
    choices: List[str] = Field(..., min_items=2, description="List of choice strings")
    answer: int = Field(..., ge=0, description="Index of correct answer (0, 1, 2, etc.)")
    explanation: Optional[str] = Field(None, description="Explanation of the answer")

class ProblemCreate(ProblemBase):
    """Schema for creating new problem"""
    pass

class ProblemUpdate(BaseModel):
    """Schema for updating problem"""
    file_uuid: Optional[UUID] = Field(None, description="File UUID this problem belongs to")
    problem_index: Optional[str] = Field(None, description="Problem index within the file")
    problem_id: Optional[str] = Field(None, description="Problem identifier")
    problem_content: Optional[str] = Field(None, description="Problem content")
    question_id: Optional[int] = Field(None, description="Question ID")
    question: Optional[str] = Field(None, description="The question text")
    choices: Optional[List[str]] = Field(None, min_items=2, description="List of choice strings")
    answer: Optional[int] = Field(None, ge=0, description="Index of correct answer")
    explanation: Optional[str] = Field(None, description="Explanation of the answer")

class ProblemResponse(ProblemBase):
    """Schema for problem response data"""
    uuid: UUID
    created_at: datetime
    updated_at: Optional[datetime]

    class Config:
        from_attributes = True
        json_schema_extra = {
            "example": {
                "uuid": "550e8400-e29b-41d4-a716-446655440000",
                "file_uuid": "550e8400-e29b-41d4-a716-446655440001",
                "problem_index": "1.1",
                "problem_id": "PROB001",
                "problem_content": "Basic arithmetic question",
                "question_id": 1,
                "question": "What is the correct value?",
                "choices": ["Option A", "Option B", "Option C"],
                "answer": 1,
                "explanation": "Option B is correct because...",
                "created_at": "2024-01-01T00:00:00Z",
                "updated_at": "2024-01-01T00:00:00Z"
            }
        }

class ProblemListResponse(BaseModel):
    """Response for problem listing"""
    problems: List[ProblemResponse]
    total_count: int
    page: int
    limit: int
    has_next: bool
    has_prev: bool

class ProblemListParams(BaseModel):
    """Query parameters for problem listing"""
    file_uuid: Optional[str] = Field(None, description="Filter by file UUID")
    search: Optional[str] = Field(None, description="Search in question text")
    page: int = Field(1, ge=1, description="Page number")
    limit: int = Field(100, ge=1, le=1000, description="Items per page")

class ProblemDetail(BaseModel):
    """Problem detail for response"""
    uuid: str = Field(..., description="Problem UUID")
    file_uuid: str = Field(..., description="File UUID")
    problem_index: Optional[Union[str, float]] = Field(None, description="Problem index")
    problem_id: Optional[str] = Field(None, description="Problem ID")
    problem_content: Optional[str] = Field(None, description="Problem content")
    question_id: Optional[int] = Field(None, description="Question ID")
    question: str = Field(..., description="Question text")
    choices: List[str] = Field(..., description="List of choices")
    answer: List[int] = Field(..., description="Correct answer indices (multi-select)")
    explanation: Optional[str] = Field(None, description="Answer explanation")


class ProblemsByFileNameListResponse(BaseModel):
    """Response for problems by file name query"""
    file_name: str = Field(..., description="File name")
    problems: List[ProblemDetail] = Field(..., description="List of problems for this file")
    file_uuid: UUID = Field(..., description="File UUID")

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
