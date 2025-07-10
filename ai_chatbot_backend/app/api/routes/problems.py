"""
Problem API routes
"""

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.orm import Session

from app.core.database import get_db
from app.schemas.problem import (
    ProblemCreate,
    ProblemUpdate,
    ProblemResponse,
    ProblemListResponse,
    ProblemListParams,
)
from app.services.problem_service import ProblemService
from app.api.deps import verify_api_token

router = APIRouter(prefix="/problems")


@router.post("/", response_model=ProblemResponse, status_code=status.HTTP_201_CREATED)
def create_problem(problem_data: ProblemCreate, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Create new problem"""
    try:
        problem = ProblemService.create_problem(db, problem_data)
        return ProblemResponse.from_orm(problem)
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )


@router.get("/", response_model=ProblemListResponse)
def list_problems(
    file_uuid: str = Query(None, description="Filter by file UUID"),
    search: str = Query(None, description="Search in question text"),
    page: int = Query(1, ge=1, description="Page number"),
    limit: int = Query(100, ge=1, le=1000, description="Items per page"),
    db: Session = Depends(get_db),
    _: bool = Depends(verify_api_token)
):
    """List problems"""
    params = ProblemListParams(
        file_uuid=file_uuid,
        search=search,
        page=page,
        limit=limit
    )
    
    result = ProblemService.list_problems(db, params)
    
    return ProblemListResponse(
        problems=[ProblemResponse.from_orm(problem) for problem in result["problems"]],
        total_count=result["total_count"],
        page=result["page"],
        limit=result["limit"],
        has_next=result["has_next"],
        has_prev=result["has_prev"],
    )


@router.get("/{problem_uuid}", response_model=ProblemResponse)
def get_problem(problem_uuid: str, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Get problem by UUID"""
    problem = ProblemService.get_problem_by_uuid(db, problem_uuid)
    if not problem:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Problem not found"
        )
    return ProblemResponse.from_orm(problem)


@router.put("/{problem_uuid}", response_model=ProblemResponse)
def update_problem(problem_uuid: str, problem_data: ProblemUpdate, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Update problem"""
    try:
        problem = ProblemService.update_problem(db, problem_uuid, problem_data)
        if not problem:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Problem not found"
            )
        return ProblemResponse.from_orm(problem)
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )


@router.delete("/{problem_uuid}", status_code=status.HTTP_204_NO_CONTENT)
def delete_problem(problem_uuid: str, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Delete problem (hard delete)"""
    success = ProblemService.delete_problem(db, problem_uuid)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Problem not found"
        ) 