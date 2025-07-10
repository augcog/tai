"""
File metadata API routes
"""

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.orm import Session

from app.core.database import get_db
from app.schemas.file_metadata import (
    FileMetadataCreate,
    FileMetadataUpdate,
    FileMetadataResponse,
    FileMetadataListResponse,
    FileMetadataListParams,
    ProblemsByFileNameListResponse,
    ProblemDetail
)
from app.services.file_metadata_service import FileMetadataService
from app.services.problem_service import ProblemService
from app.api.deps import verify_api_token

router = APIRouter(prefix="/file-metadata")


@router.post("/", response_model=FileMetadataResponse, status_code=status.HTTP_201_CREATED)
def create_file_metadata(metadata_data: FileMetadataCreate, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Create new file metadata"""
    # Check if file name already exists
    if FileMetadataService.get_file_metadata_by_name(db, metadata_data.file_name):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="File name already exists"
        )
    
    metadata = FileMetadataService.create_file_metadata(db, metadata_data)
    return FileMetadataResponse.from_orm(metadata)


@router.get("/", response_model=FileMetadataListResponse)
def list_file_metadata(
    search: str = Query(None, description="Search in file name"),
    page: int = Query(1, ge=1, description="Page number"),
    limit: int = Query(100, ge=1, le=1000, description="Items per page"),
    db: Session = Depends(get_db),
    _: bool = Depends(verify_api_token)
):
    """List file metadata with filtering and pagination"""
    params = FileMetadataListParams(
        search=search,
        page=page,
        limit=limit
    )
    
    result = FileMetadataService.list_file_metadata(db, params)
    
    return FileMetadataListResponse(
        files=[FileMetadataResponse.from_orm(file) for file in result["files"]],
        total_count=result["total_count"],
        page=result["page"],
        limit=result["limit"],
        has_next=result["has_next"],
        has_prev=result["has_prev"],
    )


@router.get("/{metadata_uuid}", response_model=FileMetadataResponse)
def get_file_metadata(metadata_uuid: str, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Get file metadata by UUID"""
    metadata = FileMetadataService.get_file_metadata_by_uuid(db, metadata_uuid)
    if not metadata:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found"
        )
    return FileMetadataResponse.from_orm(metadata)


@router.put("/{metadata_uuid}", response_model=FileMetadataResponse)
def update_file_metadata(metadata_uuid: str, metadata_data: FileMetadataUpdate, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Update file metadata information"""
    # Check if metadata exists
    existing_metadata = FileMetadataService.get_file_metadata_by_uuid(db, metadata_uuid)
    if not existing_metadata:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found"
        )
    
    # Check for file name conflicts if name is being updated
    if metadata_data.file_name and metadata_data.file_name != existing_metadata.file_name:
        if FileMetadataService.get_file_metadata_by_name(db, metadata_data.file_name):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="File name already exists"
            )
    
    metadata = FileMetadataService.update_file_metadata(db, metadata_uuid, metadata_data)
    return FileMetadataResponse.from_orm(metadata)


@router.delete("/{metadata_uuid}", status_code=status.HTTP_204_NO_CONTENT)
def delete_file_metadata(metadata_uuid: str, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Delete file metadata"""
    success = FileMetadataService.delete_file_metadata(db, metadata_uuid)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found"
        )

@router.get("/by-name/{file_name}", response_model=ProblemsByFileNameListResponse)
def get_problems_by_file_name(file_name: str, db: Session = Depends(get_db), _: bool = Depends(verify_api_token)):
    """Get problems by file name"""
    metadata = FileMetadataService.get_file_metadata_by_name(db, file_name)
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


