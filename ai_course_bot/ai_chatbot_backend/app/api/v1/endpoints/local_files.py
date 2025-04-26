from typing import List, Optional, Dict, Any
from fastapi import APIRouter, Depends, Query, Path, HTTPException, status, Request
from fastapi.responses import FileResponse, JSONResponse

from ..schemas.local_file import LocalFileListResponse, LocalFile, FileCategory, FileDirectory, FileHierarchyResponse
from ..services.file_storage import local_storage
from ...deps import get_current_user, auth_with_query_param

router = APIRouter()


@router.get("", response_model=LocalFileListResponse)
async def list_files(
    directory: Optional[str] = Query(None, description="Directory to list files from"),
    category: Optional[str] = Query(None, description="Filter files by category (Document, Assignment, Video, Others)"),
    folder: Optional[str] = Query(None, description="Filter files by folder (Lab Material, Code Script, Exams, Past Projects)"),
    course_code: Optional[str] = Query(None, description="Filter files by course code"),
    include_directories: bool = Query(False, description="Include directory information in response"),
    include_categories: bool = Query(False, description="Include category information in response"),
    user: dict = Depends(get_current_user)
):
    """
    List all files in the specified directory or in all directories if not specified.

    Can filter by category, folder, or course code.
    Optionally includes directory structure and category information.
    """
    # Get files from the storage service
    files = local_storage.list_files(directory=directory)

    # Apply filters if specified
    if category:
        files = [f for f in files if getattr(f, "category", None) == category]

    if folder:
        files = [f for f in files if getattr(f, "folder", None) == folder]

    if course_code:
        files = [f for f in files if getattr(f, "course_code", None) == course_code]

    # Convert service models to Pydantic models
    pydantic_files = [
        LocalFile(
            file_name=file.file_name,
            file_path=file.file_path,
            mime_type=file.mime_type,
            size_bytes=file.size_bytes,
            modified_time=file.modified_time,
            directory=file.directory,
            # Include additional fields if available
            thumbnail_url=getattr(file, "thumbnail_url", None),
            description=getattr(file, "description", None),
            author=getattr(file, "author", None),
            created_time=getattr(file, "created_time", None),
            last_accessed_time=getattr(file, "last_accessed_time", None),
            course_code=getattr(file, "course_code", None),
            assignment_number=getattr(file, "assignment_number", None),
            lecture_number=getattr(file, "lecture_number", None),
            category=getattr(file, "category", None),
            folder=getattr(file, "folder", None)
        ) for file in files
    ]

    # Prepare response
    response = {
        "files": pydantic_files,
        "total_count": len(pydantic_files)
    }

    # Include directories if requested
    if include_directories:
        directories = local_storage.list_directories(directory=directory)
        response["directories"] = directories

    # Include categories if requested
    if include_categories:
        # Define standard categories
        categories = [
            FileCategory(
                id="document",
                name="Document",
                icon="file-text",
                description="Course documents and materials"
            ),
            FileCategory(
                id="assignment",
                name="Assignment",
                icon="clipboard",
                description="Homework and assignment files"
            ),
            FileCategory(
                id="video",
                name="Video",
                icon="video",
                description="Lecture videos and recordings"
            ),
            FileCategory(
                id="others",
                name="Others",
                icon="file",
                description="Other course-related files"
            )
        ]
        response["categories"] = categories

    return response


@router.get("/categories", response_model=List[FileCategory])
async def list_categories(
    user: dict = Depends(get_current_user)
):
    """
    List all available file categories
    """
    # Define standard categories
    categories = [
        FileCategory(
            id="document",
            name="Document",
            icon="file-text",
            description="Course documents and materials"
        ),
        FileCategory(
            id="assignment",
            name="Assignment",
            icon="clipboard",
            description="Homework and assignment files"
        ),
        FileCategory(
            id="video",
            name="Video",
            icon="video",
            description="Lecture videos and recordings"
        ),
        FileCategory(
            id="others",
            name="Others",
            icon="file",
            description="Other course-related files"
        )
    ]
    return categories


@router.get("/folders", response_model=List[FileDirectory])
async def list_folders(
    user: dict = Depends(get_current_user)
):
    """
    List all available file folders/directories
    """
    # Define standard folders
    folders = [
        FileDirectory(
            name="Lab Material",
            path="documents/lab_material",
            is_category=True,
            icon="folder-laboratory"
        ),
        FileDirectory(
            name="Code Script",
            path="documents/code_script",
            is_category=True,
            icon="folder-code"
        ),
        FileDirectory(
            name="Exams",
            path="documents/exams",
            is_category=True,
            icon="folder-check"
        ),
        FileDirectory(
            name="Past Projects",
            path="documents/past_projects",
            is_category=True,
            icon="folder-archive"
        )
    ]
    return folders


@router.get("/hierarchy", response_model=FileHierarchyResponse)
async def get_file_hierarchy(
    directory: Optional[str] = Query(None, description="Directory to start from"),
    max_depth: int = Query(-1, description="Maximum depth to traverse (-1 for unlimited)"),
    user: dict = Depends(get_current_user)
):
    """
    Get a hierarchical tree structure of files and directories

    This endpoint provides a tree view of the file system, which is useful for:
    - Visualizing the directory structure
    - Building file browsers or tree views in the UI
    - Understanding the organization of files

    The response includes metadata about the hierarchy, such as total files,
    total directories, and maximum depth.
    """
    try:
        # Log the request
        print(f"Hierarchy request - directory: {directory}, max_depth: {max_depth}")

        # Get the file hierarchy from the storage service
        hierarchy = local_storage.get_file_hierarchy(directory=directory, max_depth=max_depth)

        # Log the response
        print(f"Hierarchy response - total files: {hierarchy.get('total_files', 0)}, total directories: {hierarchy.get('total_directories', 0)}")

        # Create the response
        response = FileHierarchyResponse(**hierarchy)

        # Return the hierarchy
        return response
    except HTTPException as e:
        # Re-raise HTTP exceptions
        print(f"HTTP exception in hierarchy endpoint: {e.detail}")
        raise e
    except Exception as e:
        # Handle unexpected errors
        print(f"Unexpected error in hierarchy endpoint: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving file hierarchy: {str(e)}"
        )


@router.options("/{file_path:path}")
async def options_file(
    file_path: str = Path(..., description="Path to the file")
):
    """
    Handle OPTIONS requests for file paths
    This is needed for CORS preflight requests from the browser
    """
    return JSONResponse(
        content={"methods": ["GET", "OPTIONS"]},
        headers={
            "Allow": "GET, OPTIONS",
            "Access-Control-Allow-Methods": "GET, OPTIONS",
            "Access-Control-Allow-Headers": "Authorization, Content-Type"
        }
    )


@router.get("/{file_path:path}", response_class=FileResponse)
async def get_file(
    request: Request,
    file_path: str = Path(..., description="Path to the file"),
    user: dict = Depends(auth_with_query_param)
):
    """
    Retrieve a file by its path

    Returns the file with appropriate content-type headers for browser rendering
    """
    try:
        # Pass the file_path parameter to the service
        return local_storage.get_file(file_path)
    except HTTPException as e:
        # Re-raise HTTP exceptions
        raise e
    except Exception as e:
        # Handle unexpected errors
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving file: {str(e)}"
        )