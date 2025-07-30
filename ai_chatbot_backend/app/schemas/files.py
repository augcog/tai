"""
Clean, simple file schemas - no over-engineering
"""

from datetime import datetime
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
import json

class FileMetadata(BaseModel):
    """Simple, clean file metadata"""

    # API fields
    uuid: str = Field(..., description="Unique file identifier")
    filename: str = Field(..., description="Original filename")
    title: Optional[str] = Field(None, description="Clean, formatted title")
    relative_path: str = Field(..., description="Path relative to data directory")

    # File properties
    size_bytes: int = Field(..., description="File size in bytes")
    mime_type: str = Field(..., description="MIME type")
    created_at: Optional[datetime] = Field(None, description="File creation timestamp")
    modified_at: Optional[datetime] = Field(
        None, description="Last modification timestamp"
    )

    # Simple metadata
    course: Optional[str] = Field(None, description="Course code (e.g., CS61A)")
    category: Optional[str] = Field(
        None, description="File category (document, video, audio, other)"
    )
    
    # Advanced metadata - now as parsed JSON
    sections: List[Section] = Field(..., description="Parsed sections of the file")
    
    # Download URL for frontend
    download_url: Optional[str] = Field(None, description="Download URL for this file")

    @classmethod
    def from_db_model(cls, db_model):
        """Create schema from database model with proper field mapping"""
        import mimetypes
        import os
        from pathlib import Path
        
        # Get file path for additional metadata
        from app.config import settings
        
        # Use the same logic as the file service
        if hasattr(settings, "DATA_DIR") and settings.DATA_DIR:
            data_dir = str(settings.DATA_DIR).split("#")[0].strip().strip("\"'")
            if os.path.isabs(data_dir):
                base_path = Path(data_dir)
            else:
                # Try relative to current working directory first
                base_path = Path(data_dir)
                if not base_path.exists():
                    # Fallback to project root
                    import inspect
                    current_file = Path(inspect.getfile(inspect.currentframe())).resolve()
                    project_root = current_file.parent.parent.parent
                    base_path = project_root / data_dir
        else:
            # Default to data directory relative to current working directory
            base_path = Path("data")
        
        file_path = base_path / db_model.relative_path
        
        # Get file stats
        size_bytes = 0
        mime_type = "application/octet-stream"
        try:
            if file_path.exists():
                size_bytes = file_path.stat().st_size
                mime_type, _ = mimetypes.guess_type(str(file_path))
                if not mime_type:
                    mime_type = "application/octet-stream"
        except:
            pass
        
        # Generate title from filename
        title = db_model.file_name
        if title:
            title = title.rsplit(".", 1)[0]  # Remove extension
            title = title.replace("_", " ").replace("-", " ")  # Replace with spaces
            title = " ".join(title.split())  # Normalize whitespace
            title = title.title()  # Title case
        
        # Parse sections JSON string to list of Section objects
        sections_data = []
        if db_model.sections:
            try:
                sections_json = json.loads(db_model.sections)
                sections_data = [Section(**section) for section in sections_json]
            except (json.JSONDecodeError, ValueError) as e:
                # Fallback to empty list if parsing fails
                sections_data = []
        
        return cls(
            uuid=str(db_model.uuid),
            filename=db_model.file_name,
            title=title,
            relative_path=db_model.relative_path,
            size_bytes=size_bytes,
            mime_type=mime_type,
            created_at=None,  # Not available in metadata db
            modified_at=None,  # Not available in metadata db
            course=db_model.course_code,
            category=None,  # Not available in metadata db
            sections=sections_data,
            download_url=f"/api/files/{db_model.uuid}/download",
        )

    class Config:
        json_schema_extra = {
            "example": {
                "uuid": "550e8400-e29b-41d4-a716-446655440000",
                "filename": "lab_01_getting_started.pdf",
                "title": "Lab 01 Getting Started",
                "relative_path": "CS61A/documents/lab_01_getting_started.pdf",
                "size_bytes": 1048576,
                "mime_type": "application/pdf",
                "created_at": "2023-01-01T00:00:00Z",
                "modified_at": "2023-01-01T00:00:00Z",
                "course": "CS61A",
                "category": "document",
                "sections": [
                    {
                        "aspects": [
                            {
                                "content": "Computer storage is organized in four tiers: CPU Registers, Main Memory (RAM), File System, and Offline Storage.",
                                "type": "Definition"
                            },
                            {
                                "content": "Files live in the third tier - the file system. While slower than RAM, files provide the crucial ability to persist data between program runs.",
                                "type": "Explanation of File System Tier"
                            },
                            {
                                "content": "Files are essential for persistent data storage, allowing programs to save and retrieve information between sessions.",
                                "type": "Purpose"
                            }
                        ],
                        "index": 3,
                        "key_concept": "Computer Storage Hierarchy and Role of File System",
                        "name": "Computer Storage Hierarchy"
                    },
                    {
                        "aspects": [
                            {
                                "content": "Python's os module provides a portable way to work with file paths across different operating systems.",
                                "type": "Definition"
                            },
                            {
                                "content": "Key functions demonstrated: os.path.abspath(), os.path.dirname(), os.getcwd(), os.path.exists(), and os.listdir().",
                                "type": "Functions"
                            },
                            {
                                "content": "The os module allows for checking and manipulating file and directory paths in a way that works across platforms.",
                                "type": "Purpose"
                            }
                        ],
                        "index": 4,
                        "key_concept": "Python's os Module and File Paths",
                        "name": "File Paths and the OS Module"
                    }
                ],
                "download_url": "/api/files/550e8400-e29b-41d4-a716-446655440000/download"
            }
        }


class FileListResponse(BaseModel):
    """Response for file listing"""

    files: List[FileMetadata] = Field(..., description="List of files")
    total_count: int = Field(..., description="Total number of files")
    page: int = Field(..., description="Current page number")
    limit: int = Field(..., description="Items per page")
    has_next: bool = Field(..., description="Whether there are more pages")
    has_prev: bool = Field(..., description="Whether there are previous pages")

    # Applied filters for transparency
    filters_applied: dict = Field(
        default_factory=dict, description="Filters that were applied"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "files": [
                    {
                        "uuid": "550e8400-e29b-41d4-a716-446655440000",
                        "filename": "lab_01.pdf",
                        "title": "Lab 01",
                        "relative_path": "CS61A/documents/lab_01.pdf",
                        "size_bytes": 1048576,
                        "mime_type": "application/pdf",
                        "created_at": "2023-01-01T00:00:00Z",
                        "modified_at": "2023-01-01T00:00:00Z",
                        "course": "CS61A",
                        "category": "document",
                    }
                ],
                "total_count": 1,
                "page": 1,
                "limit": 100,
                "has_next": False,
                "has_prev": False,
                "filters_applied": {"course": "CS61A", "category": "document"},
            }
        }


class FileStatsResponse(BaseModel):
    """Simple file statistics"""

    total_files: int = Field(..., description="Total number of files")
    base_directory: str = Field(..., description="Base directory path")
    auto_discovery: str = Field(..., description="Auto-discovery status")
    courses: dict = Field(..., description="Course breakdown with counts")
    last_updated: datetime = Field(..., description="Last update timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "total_files": 150,
                "base_directory": "/path/to/data",
                "auto_discovery": "enabled",
                "courses": {"CS61A": 75, "CS61B": 50, "CS70": 25},
                "last_updated": "2023-01-01T12:00:00Z",
            }
        }


class ErrorResponse(BaseModel):
    """Standard error response"""

    detail: str = Field(..., description="Error message")
    error_code: Optional[str] = Field(None, description="Error code")

    class Config:
        json_schema_extra = {
            "example": {"detail": "File not found", "error_code": "FILE_NOT_FOUND"}
        }


class DirectoryInfo(BaseModel):
    """Information about a directory in the file system"""
    
    name: str = Field(..., description="Directory name")
    path: str = Field(..., description="Full path relative to course root")
    file_count: int = Field(..., description="Number of files in this directory and subdirectories")
    has_subdirs: bool = Field(..., description="Whether this directory has subdirectories")
    
    class Config:
        json_schema_extra = {
            "example": {
                "name": "practice",
                "path": "Part One/practice",
                "file_count": 5,
                "has_subdirs": True
            }
        }


class BreadcrumbItem(BaseModel):
    """Breadcrumb navigation item"""
    
    name: str = Field(..., description="Display name for breadcrumb")
    path: str = Field(..., description="Path for navigation")
    
    class Config:
        json_schema_extra = {
            "example": {
                "name": "Part One",
                "path": "Part One"
            }
        }


class DirectoryBrowserResponse(BaseModel):
    """Response for directory browsing with hierarchical structure"""
    
    directories: List[DirectoryInfo] = Field(..., description="Immediate subdirectories")
    files: List[FileMetadata] = Field(..., description="Files in current directory")
    current_path: str = Field(..., description="Current directory path")
    breadcrumbs: List[BreadcrumbItem] = Field(..., description="Breadcrumb navigation")
    course_name: str = Field(..., description="Course name for context")
    
    class Config:
        json_schema_extra = {
            "example": {
                "directories": [
                    {
                        "name": "practice",
                        "path": "Part One/practice",
                        "file_count": 5,
                        "has_subdirs": False
                    }
                ],
                "files": [
                    {
                        "uuid": "550e8400-e29b-41d4-a716-446655440000",
                        "filename": "intro.pdf",
                        "title": "Introduction",
                        "relative_path": "Part One/intro.pdf",
                        "size_bytes": 1048576,
                        "mime_type": "application/pdf",
                        "course": "ROAR Academy",
                        "download_url": "/api/files/550e8400-e29b-41d4-a716-446655440000/download"
                    }
                ],
                "current_path": "Part One",
                "breadcrumbs": [
                    {"name": "Root", "path": ""},
                    {"name": "Part One", "path": "Part One"}
                ],
                "course_name": "ROAR Academy"
            }
        }

class SectionAspect(BaseModel):
    """Individual aspect within a section"""
    content: str = Field(..., description="The educational content")
    type: str = Field(..., description="Type of content (Definition, Functions, etc.)")


class Section(BaseModel):
    """Educational section with structured content"""
    aspects: List[SectionAspect] = Field(..., description="List of content aspects")
    index: int = Field(..., description="Section order/position")
    key_concept: str = Field(..., description="Main topic of the section")
    name: str = Field(..., description="Section title")

# Simple query parameters
class FileListParams(BaseModel):
    """Simple query parameters for file listing"""

    course: Optional[str] = Field(None, description="Filter by course")
    category: Optional[str] = Field(None, description="Filter by category")
    search: Optional[str] = Field(None, description="Search in filename and title")
    page: int = Field(1, ge=1, description="Page number")
    limit: int = Field(100, ge=1, le=1000, description="Items per page")

    class Config:
        json_schema_extra = {
            "example": {
                "course": "CS61A",
                "category": "document",
                "search": "lab",
                "page": 1,
                "limit": 50,
            }
        }

