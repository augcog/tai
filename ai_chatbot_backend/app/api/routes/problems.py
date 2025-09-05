"""
Problem API routes
"""

import json
from urllib.parse import unquote
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session

from app.core.dbs.metadata_db import get_metadata_db
from app.schemas.problem import ProblemsByFilePathListResponse, ProblemDetail
from app.services.file_service import file_service
from app.services.problem_service import ProblemService
from app.api.deps import verify_api_token

router = APIRouter()

"""
TODO: will be deprecated in the future, use /by-path/{file-path} instead
"""
@router.get("/by-path/{file_path}", response_model=ProblemsByFilePathListResponse, summary="Get problems by file path")
def get_problems_by_file_path(file_path: str, db: Session = Depends(get_metadata_db), _: bool = Depends(verify_api_token)):
    decoded_file_path = unquote(file_path)
    metadata = file_service.get_file_metadata_by_path(db, decoded_file_path)
    if not metadata:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found by given path " + decoded_file_path
        )

    # Get problems for this file using the new relationship
    problems = ProblemService.get_problems_by_file_uuid(db, str(metadata.uuid))
    problem_details = []
    
    for problem in problems:
        try:
            choices = json.loads(problem.choices) if problem.choices else []
            answer = json.loads(problem.answer) if problem.answer else []
        except (json.JSONDecodeError, TypeError):
            choices = []
            answer = []
        
        problem_details.append(ProblemDetail(
            uuid=str(problem.uuid),
            file_uuid=str(problem.file_uuid),
            problem_index=problem.problem_index,
            problem_id=problem.problem_id,
            problem_content=problem.problem_content,
            question_id=problem.question_id,
            question=problem.question or "",
            choices=choices,
            answer=answer,
            explanation=problem.explanation
        ))
    return ProblemsByFilePathListResponse(file_name=decoded_file_path, problems=problem_details, file_uuid=metadata.uuid)
