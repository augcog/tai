import mimetypes
from datetime import datetime
from pathlib import Path
from typing import List, Optional

from fastapi import HTTPException, status
from fastapi.responses import FileResponse

from app.config import settings

# Initialize mimetypes
mimetypes.init()

# Define base directory for file storage
FILE_BASE_DIR = Path(settings.DATA_DIR if hasattr(settings, "DATA_DIR") else "data")

# Ensure directories exist
FILE_BASE_DIR.mkdir(exist_ok=True)
for dir_name in ["documents", "videos", "audios", "others"]:
    (FILE_BASE_DIR / dir_name).mkdir(exist_ok=True)


class LocalFile:
    """Model for file information"""
    file_name: str
    file_path: str
    mime_type: str
    size_bytes: int
    modified_time: str
    directory: Optional[str] = None

    def __init__(
            self,
            file_name: str,
            file_path: str,
            mime_type: str,
            size_bytes: int,
            modified_time: str,
            directory: Optional[str] = None
    ):
        self.file_name = file_name
        self.file_path = file_path
        self.mime_type = mime_type
        self.size_bytes = size_bytes
        self.modified_time = modified_time
        self.directory = directory

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            "file_name": self.file_name,
            "file_path": self.file_path,
            "mime_type": self.mime_type,
            "size_bytes": self.size_bytes,
            "modified_time": self.modified_time,
            "directory": self.directory
        }


def list_files(directory: Optional[str] = None) -> List[LocalFile]:
    """
    List all files in the specified directory or in all directories if not specified
    """
    files = []

    # Target directory for scanning
    target_dir = FILE_BASE_DIR
    if directory:
        target_dir = FILE_BASE_DIR / directory
        if not target_dir.exists() or not target_dir.is_dir():
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Directory '{directory}' not found"
            )

    # Scan all directories if none specified
    if directory:
        dirs_to_scan = [target_dir]
    else:
        dirs_to_scan = [d for d in FILE_BASE_DIR.iterdir() if d.is_dir()]

    # Collect file information from each directory
    for dir_path in dirs_to_scan:
        dir_name = dir_path.name
        for file_path in dir_path.glob("*"):
            if file_path.is_file():
                # Get file stats
                stat = file_path.stat()
                mime_type, _ = mimetypes.guess_type(file_path)

                files.append(LocalFile(
                    file_name=file_path.name,
                    file_path=str(file_path.relative_to(FILE_BASE_DIR)),
                    mime_type=mime_type or "application/octet-stream",
                    size_bytes=stat.st_size,
                    modified_time=datetime.fromtimestamp(stat.st_mtime).isoformat(),
                    directory=dir_name
                ))

    return files


def get_file(file_path: str) -> FileResponse:
    """
    Retrieve a file by its path
    
    Args:
        file_path: Path to the file, relative to the base data directory
        
    Returns:
        FileResponse: File content with appropriate Content-Type header
    """
    # Construct absolute path and ensure it's within the base directory
    full_path = FILE_BASE_DIR / file_path

    # Security check - ensure path is within FILE_BASE_DIR
    try:
        full_path.relative_to(FILE_BASE_DIR)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: Path is outside the allowed directory"
        )

    # Check if file exists
    if not full_path.exists() or not full_path.is_file():
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"File not found: {file_path}"
        )

    # Determine mimetype
    mime_type, _ = mimetypes.guess_type(full_path)

    # Return file response
    return FileResponse(
        path=str(full_path),
        media_type=mime_type,
        filename=full_path.name
    )


# Make functions available as a module
class LocalStorageModule:
    def list_files(self, directory: Optional[str] = None) -> List[LocalFile]:
        return list_files(directory)

    def get_file(self, file_path: str) -> FileResponse:
        return get_file(file_path)

    LocalFile = LocalFile


# Module instance
local_storage = LocalStorageModule()
