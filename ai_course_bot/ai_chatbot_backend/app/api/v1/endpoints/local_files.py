from typing import Optional
from fastapi import APIRouter, Depends, Query, Path
from fastapi.responses import FileResponse

from ..schemas.local_file import LocalFileListResponse, LocalFile
from ..services.file_storage import local_storage
from ...deps import get_current_user

router = APIRouter()


@router.get("", response_model=LocalFileListResponse)
def list_files(
    directory: Optional[str] = Query(None, description="Directory to list files from"),
    user: dict = Depends(get_current_user)
):
    """
    List all files in the specified directory or in all directories if not specified
    """
    files = local_storage.list_files(directory=directory)
    
    # Convert service models to Pydantic models
    pydantic_files = [
        LocalFile(
            file_name=file.file_name,
            file_path=file.file_path,
            mime_type=file.mime_type,
            size_bytes=file.size_bytes,
            modified_time=file.modified_time,
            directory=file.directory
        ) for file in files
    ]
    
    return {
        "files": pydantic_files,
        "total_count": len(pydantic_files)
    }


@router.get("/{file_path:path}", response_class=FileResponse)
def get_file(
    file_path: str = Path(..., description="Path to the file"),
    user: dict = Depends(get_current_user)
):
    """
    Retrieve a file by its path
    """
    # Pass the file_path parameter to the service
    return local_storage.get_file(file_path)
