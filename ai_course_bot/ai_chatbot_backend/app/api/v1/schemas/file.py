from typing import List, Optional

from pydantic import BaseModel


class File(BaseModel):
    fileId: str
    name: str
    isDirectory: bool
    path: str
    updatedAt: str  # ISO formatted timestamp
    size: int
    fileType: str


class Meta(BaseModel):
    page: int
    limit: int
    total: int
    totalPages: int


class FilesResponse(BaseModel):
    data: List[File]
    meta: Meta


class FileDetailResponse(BaseModel):
    fileId: str
    name: str
    url: str
    metaData: Optional[str] = None
