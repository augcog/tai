from typing import Tuple, List, Optional

from fastapi import HTTPException, status

from ..schemas.file import File, FileDetailResponse


def get_course_files(courseId: str, folder: Optional[str], page: int, limit: int, user: dict) -> Tuple[List[File], int]:
    """
    Return a list of files for a given course.
    Raises a 404 error if the course is not found.
    """
    # For the sake of example, assume only course IDs "CS61A" and "CS61B" exist.
    if courseId not in ["CS61A", "CS61B"]:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Course not found")

    # Simulated list of files
    files = [
        File(
            fileId="file123",
            name="Pic.png",
            isDirectory=False,
            path="/Pictures/Pic.png",
            updatedAt="2024-09-08T16:45:00Z",
            size=2048,
            fileType="image"
        ),
        File(
            fileId="file124",
            name="Document.pdf",
            isDirectory=False,
            path="/Docs/Document.pdf",
            updatedAt="2024-09-07T12:30:00Z",
            size=4096,
            fileType="pdf"
        ),
    ]
    total = len(files)
    start = (page - 1) * limit
    end = start + limit
    paged_files = files[start:end]
    return paged_files, total


def get_file_detail(fileId: str, user: dict) -> Optional[FileDetailResponse]:
    """
    Return file details including an LLM-based summary.
    Returns None if the file is not found.
    """
    if fileId != "file123":
        return None
    return FileDetailResponse(
        fileId="file123",
        name="Pic.png",
        url="https://thecourseserver.com/files/Pic.png",
        metaData="LLM-based summary here."
    )
