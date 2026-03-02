"""
Module routes for listing and accessing course modules
"""

import logging
from typing import Optional
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
    course_code: str = Query(..., description="Filter by course code (e.g., CS61A)"),
    db: Session = Depends(get_metadata_db),
    _: bool = Depends(verify_api_token),
):
    """
    List all modules for a given course.

    A module is defined as:
    - Subdirectories under practice/*/ (grandchildren of practice/)
    - Subdirectories under study/ (children of study/)
    - Subdirectories under support/ (children of support/)

    Example usage:
    - GET /api/modules?course_code=CS61A - List all modules for CS61A
    """
    try:
        modules_data = module_service.list_modules(db=db, course_code=course_code)

        modules = [Module(**module_data) for module_data in modules_data]

        return ModuleListResponse(
            modules=modules,
            total_count=len(modules),
            course_code=course_code
        )

    except Exception as e:
        logger.error(f"Error listing modules: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing modules: {str(e)}",
        )


@router.get("/{module_path:path}/files", response_model=FileListResponse, summary="List files in a module")
async def list_module_files(
    module_path: str,
    course_code: str = Query(..., description="Course code (e.g., CS61A)"),
    page: int = Query(1, ge=1, description="Page number"),
    limit: int = Query(100, ge=1, le=1000, description="Items per page"),
    db: Session = Depends(get_metadata_db),
    _: bool = Depends(verify_api_token),
):
    """
    List all files within a specific module.

    Example usage:
    - GET /api/modules/practice/labs/lab01/files?course_code=CS61A - List files in lab01
    - GET /api/modules/study/week1/files?course_code=CS61A - List files in week1
    """
    try:
        # Get all files in the module (validates module_path internally)
        files = module_service.get_module_files(
            db=db,
            course_code=course_code,
            module_path=module_path
        )

        # Apply pagination
        total_count = len(files)
        start_idx = (page - 1) * limit
        end_idx = start_idx + limit
        paginated_files = files[start_idx:end_idx]

        return FileListResponse(
            files=[FileMetadata.from_db_model(file) for file in paginated_files],
            total_count=total_count,
            page=page,
            limit=limit,
            has_next=end_idx < total_count,
            has_prev=page > 1,
            filters_applied={
                "course_code": course_code,
                "module_path": module_path,
            },
        )

    except ValueError as e:
        # Invalid module path
        logger.warning(f"Invalid module path '{module_path}': {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error listing module files: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing module files: {str(e)}",
        )
