from app.api.v1.endpoints import file_completions, completions, courses, course_admin, local_files, file_tester
#TODO add summarization to import
from fastapi import APIRouter

router = APIRouter()

router.include_router(
    file_completions.router,
    prefix="/files/{fileId}/completions",
    tags=["file completions"]
)

router.include_router(
    completions.router,
    prefix="/completions",
    tags=["completions"]
)

router.include_router(
    courses.router,
    prefix="/courses",
    tags=["courses"]
)

router.include_router(
    course_admin.router,
    prefix="/course-admin",
    tags=["course-admin"]
)

router.include_router(
    local_files.router,
    prefix="/local-files",
    tags=["local-files"]
)

router.include_router(
    file_tester.router,
    prefix="/file-tester",
    tags=["file-tester"]
)

# router.include_router(
#     summarization.router,
#     prefix="/summarization",
#     tags=["summarization"]
# )
