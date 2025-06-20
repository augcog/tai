from typing import Optional
from sqlalchemy.orm import Session

from fastapi import APIRouter, Depends, Query

from ..schemas.course import CoursesResponse, Meta as CoursesMeta
from ..services import courses_service
from ...deps import get_current_user_optional
from app.core.database import get_db

router = APIRouter()


@router.get("", response_model=CoursesResponse)
def get_course_list(
        page: int = Query(1, ge=1),
        limit: int = Query(10, ge=1),
        user: Optional[dict] = Depends(get_current_user_optional),
        db: Session = Depends(get_db)
):
    courses, total = courses_service.get_courses(
        page=page, limit=limit, user=user, db=db)
    total_pages = (total + limit - 1) // limit
    return CoursesResponse(
        data=courses,
        meta=CoursesMeta(page=page, limit=limit,
                         total=total, totalPages=total_pages)
    )
