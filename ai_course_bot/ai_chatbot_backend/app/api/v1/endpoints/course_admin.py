from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
import uuid
from typing import Optional

from app.api.v1.schemas.course_admin import CourseCreate, CourseUpdate, CourseResponse, CourseListResponse
from app.api.v1.services import course_admin_service
from app.core.database import get_db
from app.api.deps import get_admin_token_user

router = APIRouter()


@router.post("/", response_model=CourseResponse, status_code=status.HTTP_201_CREATED)
def create_course(course: CourseCreate, db: Session = Depends(get_db), admin_user: dict = Depends(get_admin_token_user)):
    """Create a new course. Requires admin token authentication."""
    return course_admin_service.create_course(db, course)


@router.get("/", response_model=CourseListResponse)
def read_courses(skip: int = 0, limit: int = 100, db: Session = Depends(get_db)):
    """Get all courses with pagination. No authentication required."""
    courses, total = course_admin_service.get_courses(
        db, skip=skip, limit=limit)
    return CourseListResponse(courses=courses, total=total)


@router.get("/{course_id}", response_model=CourseResponse)
def read_course(course_id: int, db: Session = Depends(get_db)):
    """Get a course by ID. No authentication required."""
    db_course = course_admin_service.get_course(db, course_id)
    if db_course is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Course not found"
        )
    return db_course


@router.put("/{course_id}", response_model=CourseResponse)
def update_course(course_id: int, course: CourseUpdate, db: Session = Depends(get_db), admin_user: dict = Depends(get_admin_token_user)):
    """Update a course by ID. Requires admin token authentication."""
    updated_course = course_admin_service.update_course(db, course_id, course)
    if updated_course is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Course not found"
        )
    return updated_course


@router.delete("/{course_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_course(course_id: int, db: Session = Depends(get_db), admin_user: dict = Depends(get_admin_token_user)):
    """Delete a course by ID. Requires admin token authentication."""
    success = course_admin_service.delete_course(db, course_id)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Course not found"
        )
    return None


@router.patch("/{course_id}/toggle", response_model=CourseResponse)
def toggle_course_enabled(course_id: int, db: Session = Depends(get_db), admin_user: dict = Depends(get_admin_token_user)):
    """Toggle the enabled status of a course. Requires admin token authentication."""
    updated_course = course_admin_service.toggle_course_enabled(db, course_id)
    if updated_course is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Course not found"
        )
    return updated_course
