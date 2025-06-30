from app.api.routes import (
    completions,
    completions_v2,
    courses,
    file_completions,
    file_tester,
    files,
)
from fastapi import APIRouter

# Create main API router
api_router = APIRouter()


# V1 style completions endpoint
api_router.include_router(completions.router, prefix="/chat", tags=["completions"])

# V2 style completions endpoint
api_router.include_router(
    completions_v2.router, prefix="/chat", tags=["completions-v2"]
)

# File completions
api_router.include_router(
    file_completions.router,
    prefix="/files/{fileId}/completions",
    tags=["file-completions"],
)

# Courses management
api_router.include_router(courses.router, prefix="/courses", tags=["courses"])

# Files management
api_router.include_router(files.router, prefix="/files", tags=["files"])

# File tester
api_router.include_router(
    file_tester.router,
    prefix="/file-tester",
)
