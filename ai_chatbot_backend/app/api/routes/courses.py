from sqlalchemy.orm import Session
from fastapi import APIRouter, Depends, Query
from app.schemas.course import CoursesResponse, Meta as CoursesMeta
from app.services import courses_service
from app.api.deps import verify_api_token
from app.core.database import get_db

router = APIRouter()


@router.get("", response_model=CoursesResponse)
def get_course_list(
    page: int = Query(1, ge=1),
    limit: int = Query(10, ge=1),
    db: Session = Depends(get_db),
    _: bool = Depends(verify_api_token),
):
    courses, total = courses_service.get_courses(page=page, limit=limit, db=db)
    total_pages = (total + limit - 1) // limit
    return CoursesResponse(
        data=courses,
        meta=CoursesMeta(page=page, limit=limit, total=total, totalPages=total_pages),
    )
