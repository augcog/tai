from typing import Optional

from fastapi import APIRouter, Depends, Query, Path

from ..schemas.course import CoursesResponse, Meta as CoursesMeta
from ..schemas.file import FilesResponse, Meta as FilesMeta
from ..services import courses_service
from ..services import files_service
from ...deps import get_current_user, get_current_user_optional

router = APIRouter()


@router.get("", response_model=CoursesResponse)
def get_course_list(
        page: int = Query(1, ge=1),
        limit: int = Query(10, ge=1),
        user: Optional[dict] = Depends(get_current_user_optional)
):
    courses, total = courses_service.get_courses(page=page, limit=limit, user=user)
    total_pages = (total + limit - 1) // limit
    return CoursesResponse(
        data=courses,
        meta=CoursesMeta(page=page, limit=limit, total=total, totalPages=total_pages)
    )


@router.get("/{courseId}/files", response_model=FilesResponse)
def get_course_files(
        courseId: str = Path(...),
        folder: str = Query(None),
        page: int = Query(1, ge=1),
        limit: int = Query(10, ge=1),
        user: dict = Depends(get_current_user)
):
    files, total = files_service.get_course_files(courseId=courseId, folder=folder, page=page, limit=limit, user=user)
    total_pages = (total + limit - 1) // limit
    return FilesResponse(
        data=files,
        meta=FilesMeta(page=page, limit=limit, total=total, totalPages=total_pages)
    )
