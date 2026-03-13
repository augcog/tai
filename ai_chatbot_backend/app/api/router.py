from app.api.routes import (
    completions,
    courses,
    files,
    modules,
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

# Modules management
api_router.include_router(modules.router, prefix="/modules", tags=["modules"])

# Problems management
api_router.include_router(
    problems.router, prefix="/problems", tags=["problems"])
