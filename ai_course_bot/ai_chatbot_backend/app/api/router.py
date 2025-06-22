from fastapi import APIRouter
from app.api.routes import auth, completions, courses, files, admin, file_completions, file_tester, completions_v2

# Create main API router
api_router = APIRouter()

# Authentication endpoints
api_router.include_router(
    auth.router,
    prefix="/auth",
    tags=["authentication"]
)

# V1 style completions endpoint
api_router.include_router(
    completions.router,
    prefix="/chat",
    tags=["completions"]
)

# V2 style completions endpoint
api_router.include_router(
    completions_v2.router,
    prefix="/chat",
    tags=["completions-v2"]
)

# File completions
api_router.include_router(
    file_completions.router,
    prefix="/files/{fileId}/completions",
    tags=["file-completions"]
)

# Courses management
api_router.include_router(
    courses.router,
    prefix="/courses",
    tags=["courses"]
)

# Course admin
api_router.include_router(
    admin.router,
    prefix="/course-admin",
    tags=["course-admin"]
)

# Files management
api_router.include_router(
    files.router,
    prefix="/files",
    tags=["files"]
)

# File tester
api_router.include_router(
    file_tester.router,
    prefix="/file-tester",
    tags=["file-tester"]
)
