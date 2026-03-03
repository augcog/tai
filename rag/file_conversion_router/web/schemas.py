"""Pydantic schemas for batch upload API request/response validation.

This module defines all data models used by the batch upload API endpoints
for request validation and response serialization.
"""

from datetime import datetime
from enum import Enum
from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field


class JobStatus(str, Enum):
    """Possible states for a batch conversion job."""
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class FileStatus(str, Enum):
    """Possible states for individual file processing."""
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


class ProgressEventType(str, Enum):
    """Types of SSE progress events."""
    JOB_START = "job_start"
    FILE_START = "file_start"
    FILE_DONE = "file_done"
    FILE_ERROR = "file_error"
    JOB_COMPLETE = "job_complete"


# ========================
# Request Models
# ========================

class BatchUploadRequest(BaseModel):
    """Request parameters for batch file upload (sent as form data)."""
    course_code: str = Field(..., description="Course identifier (e.g., 'CS61A')")
    course_name: str = Field(..., description="Full course name")
    auto_embed: bool = Field(True, description="Whether to generate embeddings after conversion")


# ========================
# Response Models
# ========================

class FileInfo(BaseModel):
    """Information about an uploaded file."""
    file_name: str
    file_size: int
    content_type: Optional[str] = None


class BatchUploadResponse(BaseModel):
    """Response returned after files are uploaded and job is created."""
    job_id: str = Field(..., description="Unique identifier for the batch job")
    status: JobStatus = Field(..., description="Current job status")
    files_received: int = Field(..., description="Number of files received")
    files_info: List[FileInfo] = Field(default_factory=list, description="Details of received files")
    message: str = Field(..., description="Status message")


class FileError(BaseModel):
    """Error details for a failed file."""
    file_name: str
    error_message: str


class FileResult(BaseModel):
    """Result for a single file conversion."""
    file_name: str
    file_uuid: Optional[str] = None
    chunks_count: int = 0
    status: FileStatus
    error: Optional[str] = None


class BatchJobStatus(BaseModel):
    """Complete status of a batch conversion job."""
    job_id: str
    status: JobStatus
    total_files: int = 0
    processed_files: int = 0
    successful_files: int = 0
    failed_files: int = 0
    skipped_files: int = 0
    current_file: Optional[str] = None
    errors: List[FileError] = Field(default_factory=list)
    results: List[FileResult] = Field(default_factory=list)
    created_at: datetime
    updated_at: datetime
    completed_at: Optional[datetime] = None

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class BatchConversionResult(BaseModel):
    """Final result of a completed batch conversion."""
    job_id: str
    status: JobStatus
    files_processed: int
    files_failed: int
    files_skipped: int
    total_chunks: int
    errors: List[FileError] = Field(default_factory=list)
    results: List[FileResult] = Field(default_factory=list)
    duration_seconds: float
    embedding_error: Optional[str] = None


# ========================
# SSE Progress Event Models
# ========================

class ProgressEvent(BaseModel):
    """SSE progress event payload."""
    event_type: ProgressEventType
    job_id: str
    timestamp: float = Field(default_factory=lambda: datetime.now().timestamp())

    # Job-level fields
    total_files: Optional[int] = None
    processed_files: Optional[int] = None
    successful_files: Optional[int] = None
    failed_files: Optional[int] = None

    # File-level fields
    file_name: Optional[str] = None
    file_status: Optional[FileStatus] = None
    error_message: Optional[str] = None

    # Completion fields
    result: Optional[BatchConversionResult] = None

    def to_sse(self) -> str:
        """Format as SSE event string."""
        return f"event: {self.event_type.value}\ndata: {self.model_dump_json()}\n\n"


# ========================
# Validation Error Models
# ========================

class ValidationError(BaseModel):
    """Validation error for a single file."""
    file_name: str
    error_type: str  # "invalid_extension", "file_too_large", "duplicate"
    message: str


class BatchValidationResult(BaseModel):
    """Result of validating uploaded files before processing."""
    valid_files: List[str]
    invalid_files: List[ValidationError]
    total_size_bytes: int
    is_valid: bool
