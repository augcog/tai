from typing import List, Optional, Literal, Dict, Any, ForwardRef
from datetime import datetime
from pydantic import BaseModel, Field


class FileCategory(BaseModel):
    """Category model for organizing files"""
    id: str
    name: str
    icon: Optional[str] = None
    description: Optional[str] = None


class FileDirectory(BaseModel):
    """Directory model for hierarchical file organization"""
    name: str
    path: str
    parent_path: Optional[str] = None
    is_category: bool = False
    icon: Optional[str] = None


class FileNode(BaseModel):
    """Node in a file hierarchy tree"""
    name: str
    path: str
    type: str  # "file" or "directory"
    mime_type: Optional[str] = None
    size_bytes: Optional[int] = None
    modified_time: Optional[str] = None
    children: Optional[List["FileNode"]] = None

    class Config:
        schema_extra = {
            "example": {
                "name": "documents",
                "path": "documents",
                "type": "directory",
                "children": [
                    {
                        "name": "lecture1.pdf",
                        "path": "documents/lecture1.pdf",
                        "type": "file",
                        "mime_type": "application/pdf",
                        "size_bytes": 1048576,
                        "modified_time": "2023-01-01T12:00:00"
                    }
                ]
            }
        }

# Update forward references
FileNode.model_rebuild()


class LocalFile(BaseModel):
    """Model for file metadata"""
    file_name: str
    file_path: str
    mime_type: str
    size_bytes: int
    modified_time: str
    directory: Optional[str] = None

    # Additional metadata for specific file types
    thumbnail_url: Optional[str] = None
    description: Optional[str] = None
    author: Optional[str] = None
    created_time: Optional[str] = None
    last_accessed_time: Optional[str] = None

    # Fields for course-specific files
    course_code: Optional[str] = None
    assignment_number: Optional[int] = None
    lecture_number: Optional[int] = None

    # Category information (based on frontend requirements)
    category: Optional[Literal["Document", "Assignment", "Video", "Others"]] = None

    # Folder information (based on frontend requirements)
    folder: Optional[Literal["Lab Material", "Code Script", "Exams", "Past Projects"]] = None

    class Config:
        schema_extra = {
            "example": {
                "file_name": "lecture1.pdf",
                "file_path": "documents/CS61A/lecture1.pdf",
                "mime_type": "application/pdf",
                "size_bytes": 1048576,
                "modified_time": "2023-01-01T12:00:00",
                "directory": "documents",
                "category": "Document",
                "folder": "Lab Material"
            }
        }


class LocalFileListResponse(BaseModel):
    """Response model for file listing"""
    files: List[LocalFile] = []
    total_count: int = 0
    directories: Optional[List[FileDirectory]] = None
    categories: Optional[List[FileCategory]] = None

    class Config:
        schema_extra = {
            "example": {
                "files": [
                    {
                        "file_name": "lecture1.pdf",
                        "file_path": "documents/CS61A/lecture1.pdf",
                        "mime_type": "application/pdf",
                        "size_bytes": 1048576,
                        "modified_time": "2023-01-01T12:00:00",
                        "directory": "documents",
                        "category": "Document",
                        "folder": "Lab Material"
                    }
                ],
                "total_count": 1,
                "directories": [
                    {
                        "name": "Lab Material",
                        "path": "documents/CS61A/lab",
                        "parent_path": "documents/CS61A",
                        "is_category": False,
                        "icon": "folder"
                    }
                ],
                "categories": [
                    {
                        "id": "document",
                        "name": "Document",
                        "icon": "file-text",
                        "description": "Course documents and materials"
                    }
                ]
            }
        }


class FileHierarchyResponse(BaseModel):
    """Response model for file hierarchy"""
    root: FileNode
    total_files: int = 0
    total_directories: int = 0
    max_depth: int = 0

    class Config:
        schema_extra = {
            "example": {
                "root": {
                    "name": "root",
                    "path": "",
                    "type": "directory",
                    "children": [
                        {
                            "name": "documents",
                            "path": "documents",
                            "type": "directory",
                            "children": [
                                {
                                    "name": "lecture1.pdf",
                                    "path": "documents/lecture1.pdf",
                                    "type": "file",
                                    "mime_type": "application/pdf",
                                    "size_bytes": 1048576,
                                    "modified_time": "2023-01-01T12:00:00"
                                }
                            ]
                        }
                    ]
                },
                "total_files": 1,
                "total_directories": 2,
                "max_depth": 2
            }
        }