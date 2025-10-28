"""
Clean, simple file schemas - no over-engineering
"""

from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field
import json
import logging

logger = logging.getLogger(__name__)


class SectionAspect(BaseModel):
    """Individual aspect within a section"""
    content: str = Field(..., description="The educational content")
    type: str = Field(..., description="Type of content (Definition, Functions, etc.)")



class Section(BaseModel):
    """Educational section with structured content"""
    aspects: List[SectionAspect] = Field(..., description="List of content aspects")
    index: float = Field(..., description="Section order/position")
    key_concept: str = Field(..., description="Main topic of the section")
    name: str = Field(..., description="Section title")
    checking_questions: Optional[List[str]] = Field(None, description="List of checking questions")
    comprehensive_questions: Optional[List[str]] = Field(None, description="List of comprehensive questions")



class TranscriptSegment(BaseModel):
    """Video transcript segment"""
    start_time: float = Field(..., description="Start time in seconds")
    end_time: float = Field(..., description="End time in seconds") 
    speaker: str = Field(..., description="Speaker identifier (title-1, title-2, instructor, etc.)")
    text_content: str = Field(..., description="Transcript text content")


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
    
    # URLs
    download_url: Optional[str] = Field(None, description="Download URL for this file")
    original_url: Optional[str] = Field(None, description="Original source URL where the file was collected from")

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
                
                # Handle case where sections is wrapped in {"sections": [...]}
                if isinstance(sections_json, dict) and "sections" in sections_json:
                    logger.debug(f"Unwrapping nested sections structure for file {db_model.file_name}")
                    sections_json = sections_json["sections"]
                
                # Ensure sections_json is a list
                if sections_json and isinstance(sections_json, list):
                    logger.debug(f"Parsing {len(sections_json)} sections for file {db_model.file_name}")
                    for i, section in enumerate(sections_json):
                        try:
                            # Log the keys to understand the structure
                            if isinstance(section, dict):
                                logger.debug(f"Section #{i} keys: {list(section.keys())[:10]}")
                            sections_data.append(Section(**section))
                        except Exception as e:
                            logger.error(f"Failed to parse section #{i} for file {db_model.file_name}: {e}")
                            logger.error(f"Section data: {json.dumps(section, indent=2) if isinstance(section, dict) else section}")
                            raise
                elif sections_json and isinstance(sections_json, dict):
                    # If it's still a dict, log it for debugging
                    logger.warning(f"Sections is a dict, not a list for file {db_model.file_name}: {list(sections_json.keys())}")
                    sections_data = []
                else:
                    sections_data = []
            except (json.JSONDecodeError, ValueError) as e:
                logger.error(f"Failed to parse sections JSON for file {db_model.file_name}: {e}")
                logger.error(f"Raw sections data: {db_model.sections[:500] if db_model.sections else None}")
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
            original_url=db_model.url,
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
                ],
                "download_url": "/api/files/550e8400-e29b-41d4-a716-446655440000/download",
                "original_url": "https://example.com/course/cs61a/lab01.pdf"
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
                        ],
                        "download_url": "/api/files/550e8400-e29b-41d4-a716-446655440000/download",
                        "original_url": "https://example.com/course/cs61a/lab01.pdf"
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
    course_code: str = Field(..., description="Course code for context")
    
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
                            ],
                        "download_url": "/api/files/550e8400-e29b-41d4-a716-446655440000/download",
                        "original_url": "https://example.com/roar-academy/part-one/intro.pdf"
                    }
                ],
                "current_path": "Part One",
                "breadcrumbs": [
                    {"name": "Root", "path": ""},
                    {"name": "Part One", "path": "Part One"}
                ],
                "course_code": "CS61A"
            }
        }
