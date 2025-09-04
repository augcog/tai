from app.api.routes import (
    completions,
    courses,
    files,
    problems,
)
from fastapi import APIRouter

# Create main API router
api_router = APIRouter()


# Chat completions endpoint
api_router.include_router(
    completions.router, prefix="/chat", tags=["chat-completions"])

# Courses management
api_router.include_router(courses.router, prefix="/courses", tags=["courses"])

# Files management
api_router.include_router(files.router, prefix="/files", tags=["files"])

# Problems management
api_router.include_router(
    problems.router, prefix="/problems", tags=["problems"])
