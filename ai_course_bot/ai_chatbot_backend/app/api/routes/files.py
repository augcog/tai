from typing import Optional
from uuid import UUID

from fastapi import APIRouter, Depends, HTTPException, Query, Path, status
from sqlalchemy.orm import Session

from app.api.deps import get_current_user_optional, get_db
from app.schemas.files import FileMetadata, FileListResponse, FileStatsResponse
from app.services.file_service import file_service

router = APIRouter()


@router.get("", response_model=FileListResponse, summary="List files with auto-discovery")
async def list_files(
        course_code: Optional[str] = Query(
            None, description="Filter by course code (e.g., CS61A)"),
        category: Optional[str] = Query(
            None, description="Filter by category (document, video, audio, other)"),
        search: Optional[str] = Query(
            None, description="Search in file names and titles"),
        page: int = Query(1, ge=1, description="Page number"),
        limit: int = Query(100, ge=1, le=1000, description="Items per page"),
        db: Session = Depends(get_db),
        user: dict = Depends(get_current_user_optional)
):
    """
    List files with simple filtering and auto-discovery.

    Features:
    - Auto-discovery: New files found automatically, no manual rescans needed
    - Simple filtering: By course, category, search terms
    - Pagination: Efficient handling of large file collections

    Example usage:
    - GET /v1/files - List all files
    - GET /v1/files?course_code=CS61A - CS61A files only
    - GET /v1/files?category=document - Document files
    - GET /v1/files?search=lab - Search for files containing "lab"
    """
    try:
        result = file_service.list_files(
            db=db,
            course_code=course_code,
            category=category,
            search=search,
            page=page,
            limit=limit
        )

        return FileListResponse(
            files=[FileMetadata.from_db_model(file)
                   for file in result['files']],
            total_count=result['total_count'],
            page=result['page'],
            limit=result['limit'],
            has_next=result['has_next'],
            has_prev=result['has_prev'],
            filters_applied={
                "course_code": course_code,
                "category": category,
                "search": search
            }
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing files: {str(e)}"
        )


@router.get("/{file_id}", response_model=FileMetadata, summary="Get file metadata by UUID")
async def get_file_metadata(
        file_id: UUID = Path(..., description="File UUID"),
        db: Session = Depends(get_db),
        user: dict = Depends(get_current_user_optional)
):
    """
    Get detailed metadata for a specific file by its UUID.

    Returns comprehensive file information including:
    - Basic info: name, size, type, creation date
    - Academic info: course, category, assignment number
    - Access info: download count, last accessed
    """
    file_record = file_service.get_file_by_id(db, file_id)
    if not file_record:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"File with ID {file_id} not found"
        )

    return FileMetadata.from_db_model(file_record)


@router.get("/{file_id}/download", summary="Download file by UUID")
async def download_file(
        file_id: UUID = Path(..., description="File UUID"),
        db: Session = Depends(get_db),
        user: dict = Depends(get_current_user_optional)
):
    """
    Download a file by its UUID.

    Features:
    - Secure access: UUID-based, no path exposure
    - Access tracking: Download count and last access time
    - Security validation: Ensures file is within allowed directory
    - Proper headers: Correct MIME type and filename
    """
    return file_service.get_file_content(db, file_id)


@router.get("/stats/summary", response_model=FileStatsResponse, summary="Get file system statistics")
async def get_file_stats(
        db: Session = Depends(get_db),
        user: dict = Depends(get_current_user_optional)
):
    """
    Get comprehensive file system statistics.

    Returns:
    - Total file count
    - Files by course
    - System configuration
    - Last update time
    """
    try:
        stats = file_service.get_stats(db)
        return FileStatsResponse(**stats)

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting stats: {str(e)}"
        )
