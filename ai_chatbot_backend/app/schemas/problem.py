"""
Problem schemas for API requests and responses
"""

from typing import List, Optional, Union
from pydantic import BaseModel, Field
from uuid import UUID

class ProblemDetail(BaseModel):
    """Problem detail for response"""
    uuid: str = Field(..., description="Problem UUID")
    file_uuid: str = Field(..., description="File UUID")
    problem_index: Optional[Union[str, float]] = Field(None, description="Problem index")
    problem_id: Optional[str] = Field(None, description="Problem ID")
    problem_content: Optional[str] = Field(None, description="Problem content")
    question_id: Optional[str] = Field(None, description="Question ID")
    question: str = Field(..., description="Question text")
    choices: List[str] = Field(..., description="List of choices")
    answer: List[int] = Field(..., description="Correct answer indices (multi-select)")
    explanation: Optional[str] = Field(None, description="Answer explanation")
    question_type: Optional[str] = Field(None, description="Question type")


class ProblemsByFilePathListResponse(BaseModel):
    """Response for problems by file path query"""
    file_path: str = Field(..., description="File path")
    problems: List[ProblemDetail] = Field(..., description="List of problems for this file")
    file_uuid: UUID = Field(..., description="File UUID")

    class Config:
        json_schema_extra = {
            "example": {
                "file_path": "CS 61A/test_file.py",
                "problems": [
                    {
                        "uuid": "550e8400-e29b-41d4-a716-446655440001",
                        "file_uuid": "550e8400-e29b-41d4-a716-446655440000",
                        "problem_index": "1.1",
                        "problem_id": "PROB001",
                        "problem_content": "content of problem",
                        "question_id": "question_1",
                        "question": "What is the capital of France?",
                        "choices": ["Berlin", "Paris", "London"],
                        "answer": [0, 2],
                        "explanation": "Paris is the capital of France.",
                        "question_type": "regular"
                    },
                    {
                        "uuid": "550e8400-e29b-41d4-a716-446655440002",
                        "file_uuid": "550e8400-e29b-41d4-a716-446655440000",
                        "problem_index": "1.2",
                        "problem_id": "PROB002",
                        "problem_content": "content of problem",
                        "question_id": "question_2",
                        "question": "What is 2 + 2?",
                        "choices": ["3", "4", "5"],
                        "answer": [0, 1],
                        "explanation": "2 + 2 = 4.",
                        "question_type": "regular"
                    }
                ]
            }
        }
