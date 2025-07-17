"""
Problem API routes
"""

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.orm import Session

from app.core.database import get_db
from app.schemas.problem import ProblemsByFileNameListResponse
from app.services.file_service import FileService
from app.services.problem_service import ProblemService
from app.api.deps import verify_api_token

router = APIRouter(prefix="/problems")

@router.get("/by-name/{file_name}", response_model=ProblemsByFileNameListResponse)
def get_problems_by_file_name(file_name: str, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Get problems by file name"""
    metadata = FileService.get_file_metadata_by_name(db, file_name)
    if not metadata:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found by given name " + file_name
        )

    # Get problems for this file using the new relationship
    problems = ProblemService.get_problems_by_file_uuid(db, str(metadata.uuid))
    problem_details = [
        ProblemDetail(
            uuid=str(problem.uuid),
            file_uuid=str(problem.file_uuid),
            problem_index=problem.problem_index,
            problem_id=problem.problem_id,
            problem_content=problem.problem_content,
            question_id=problem.question_id,
            question=problem.question,
            choices=problem.choices,
            answer=problem.answer,
            explanation=problem.explanation
        )
        for problem in problems
    ]
    return ProblemsByFileNameListResponse(file_name=file_name, problems=problem_details)
