"""
Module routes for listing and accessing course modules
"""

import logging
from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.orm import Session

from app.core.dbs.metadata_db import get_metadata_db
from app.api.deps import verify_api_token
from app.schemas.modules import Module, ModuleListResponse
from app.schemas.files import FileMetadata, FileListResponse
from app.services.module_service import module_service

router = APIRouter()
logger = logging.getLogger(__name__)


@router.get("", response_model=ModuleListResponse, summary="List all modules")
async def list_modules(
    course_code: str = Query(..., description="Course code (e.g., CS61A)"),
    db: Session = Depends(get_metadata_db),
    _: bool = Depends(verify_api_token),
):
    """
    List all modules for a given course.

    A module is a directory grouping following these patterns:
    - practice/*/<name>  (grandchildren of practice/)
    - study/<name>       (children of study/)
    - support/<name>     (children of support/)
    """
    try:
        modules = module_service.list_modules(db=db, course_code=course_code)
        return ModuleListResponse(
            modules=[Module.from_db_model(m) for m in modules],
            total_count=len(modules),
            course_code=course_code,
        )
    except Exception as e:
        logger.error(f"Error listing modules: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing modules: {str(e)}",
        )


@router.get("/{module_uuid}/files", response_model=FileListResponse, summary="List files in a module")
async def list_module_files(
    module_uuid: str,
    page: int = Query(1, ge=1, description="Page number"),
    limit: int = Query(100, ge=1, le=1000, description="Items per page"),
    db: Session = Depends(get_metadata_db),
    _: bool = Depends(verify_api_token),
):
    """
    List all files within a specific module identified by its UUID.

    Example:
    - GET /api/modules/550e8400-e29b-41d4-a716-446655440000/files
    """
    try:
        files = module_service.get_module_files(db=db, module_uuid=module_uuid)
    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=str(e))
    except Exception as e:
        logger.error(f"Error listing module files: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing module files: {str(e)}",
        )

    total_count = len(files)
    start_idx = (page - 1) * limit
    end_idx = start_idx + limit

    return FileListResponse(
        files=[FileMetadata.from_db_model(f) for f in files[start_idx:end_idx]],
        total_count=total_count,
        page=page,
        limit=limit,
        has_next=end_idx < total_count,
        has_prev=page > 1,
        filters_applied={"module_uuid": module_uuid},
    )
