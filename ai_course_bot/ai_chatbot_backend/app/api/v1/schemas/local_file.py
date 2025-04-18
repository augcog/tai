from typing import List, Optional
from pydantic import BaseModel


class LocalFile(BaseModel):
    """Local file information model"""
    file_name: str
    mime_type: str
    size_bytes: int
    modified_time: str
    file_path: str
    directory: Optional[str] = None


class LocalFileListResponse(BaseModel):
    """Response model for local file listing"""
    files: List[LocalFile]
    total_count: int 