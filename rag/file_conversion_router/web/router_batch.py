"""Batch upload API endpoints.

This module provides the FastAPI router for batch file upload,
conversion, and progress streaming via SSE.

Endpoints:
- POST /upload: Upload files and start batch conversion
- GET /{job_id}/stream: Stream progress updates via SSE
- GET /{job_id}/status: Get current job status (polling fallback)
- POST /{job_id}/cancel: Cancel a running job
- GET /jobs: List recent jobs
"""

import asyncio
import logging
from pathlib import Path
from typing import List, Optional

from fastapi import APIRouter, UploadFile, File, Form, HTTPException, BackgroundTasks
from fastapi.responses import StreamingResponse

from file_conversion_router.config import (
    get_course_db_path,
    get_course_output_dir,
)
from file_conversion_router.web.schemas import (
    BatchUploadResponse,
    BatchJobStatus,
    BatchConversionResult,
    FileInfo,
    JobStatus,
    ProgressEvent,
    ProgressEventType,
)
from file_conversion_router.services.temp_storage_service import get_temp_storage_service
from file_conversion_router.services.batch_upload_service import (
    get_job_manager,
    get_batch_processor,
    get_file_validator,
)

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/upload", response_model=BatchUploadResponse)
async def upload_batch(
    background_tasks: BackgroundTasks,
    files: List[UploadFile] = File(..., description="Files to upload and convert"),
    course_code: str = Form(..., description="Course identifier (e.g., 'CS61A')"),
    course_name: str = Form(..., description="Full course name"),
    auto_embed: bool = Form(True, description="Generate embeddings after conversion"),
    output_dir: Optional[str] = Form(None, description="Custom output directory (optional)"),
    db_path: Optional[str] = Form(None, description="Custom database path (optional)"),
):
    """
    Upload files and start batch conversion.

    This endpoint:
    1. Validates uploaded files (extension, size)
    2. Saves files to temporary storage
    3. Creates a batch job
    4. Starts background processing
    5. Returns job ID for progress tracking

    Use the returned job_id to:
    - Stream progress: GET /batch/{job_id}/stream
    - Poll status: GET /batch/{job_id}/status
    - Cancel job: POST /batch/{job_id}/cancel
    """
    if not files:
        raise HTTPException(status_code=400, detail="No files provided")

    # Validate files
    validator = get_file_validator()
    validation_result = validator.validate_batch(files)

    if not validation_result.valid_files:
        raise HTTPException(
            status_code=400,
            detail={
                "message": "No valid files in batch",
                "errors": [e.model_dump() for e in validation_result.invalid_files]
            }
        )

    # Filter to only valid files
    valid_files = [f for f in files if f.filename in validation_result.valid_files]

    # Create job
    job_manager = get_job_manager()
    job_id = job_manager.create_job(
        course_code=course_code,
        course_name=course_name,
        file_count=len(valid_files),
    )

    # Save files to temp storage
    temp_storage = get_temp_storage_service()
    saved_paths = await temp_storage.save_uploaded_files(
        job_id=job_id,
        files=valid_files,
        preserve_paths=True,
    )

    # Determine output directory and database path
    if output_dir:
        final_output_dir = Path(output_dir)
    else:
        final_output_dir = get_course_output_dir(course_code)

    if db_path:
        final_db_path = Path(db_path)
    else:
        final_db_path = get_course_db_path(course_code)

    # Start background processing
    async def run_batch_processing():
        processor = get_batch_processor()
        try:
            await processor.process_batch(
                job_id=job_id,
                file_paths=saved_paths,
                course_code=course_code,
                course_name=course_name,
                output_dir=final_output_dir,
                db_path=final_db_path,
                auto_embed=auto_embed,
            )
        finally:
            # Schedule temp file cleanup
            await temp_storage.schedule_cleanup(job_id)

    background_tasks.add_task(run_batch_processing)

    # Build response
    files_info = []
    for f in valid_files:
        files_info.append(FileInfo(
            file_name=f.filename,
            file_size=f.size or 0,
            content_type=f.content_type,
        ))

    return BatchUploadResponse(
        job_id=job_id,
        status=JobStatus.PENDING,
        files_received=len(valid_files),
        files_info=files_info,
        message=f"Batch job created. {len(valid_files)} files queued for processing."
        + (f" {len(validation_result.invalid_files)} files rejected." if validation_result.invalid_files else ""),
    )


@router.get("/{job_id}/stream")
async def stream_progress(job_id: str):
    """
    Stream real-time progress updates via Server-Sent Events (SSE).

    Connect to this endpoint to receive live updates as files are processed.

    Event types:
    - job_start: Job started processing
    - file_start: Started processing a file
    - file_done: File completed successfully
    - file_error: File failed with error
    - job_complete: All files processed

    The connection will close automatically when the job completes.
    """
    job_manager = get_job_manager()

    # Check job exists
    job = job_manager.get_job_status(job_id)
    if not job:
        raise HTTPException(status_code=404, detail=f"Job not found: {job_id}")

    # Get progress queue
    queue = job_manager.get_progress_queue(job_id)
    if not queue:
        raise HTTPException(status_code=404, detail=f"Progress queue not found for job: {job_id}")

    async def event_generator():
        """Generate SSE events from the progress queue."""
        try:
            while True:
                try:
                    # Wait for next event with timeout for keepalive
                    event = await asyncio.wait_for(queue.get(), timeout=30.0)

                    # Yield the event
                    yield event.to_sse()

                    # Check if job is complete
                    if event.event_type == ProgressEventType.JOB_COMPLETE:
                        break

                except asyncio.TimeoutError:
                    # Send keepalive comment
                    yield ": keepalive\n\n"

                    # Check if job is still running
                    current_status = job_manager.get_job_status(job_id)
                    if current_status and current_status.status in [
                        JobStatus.COMPLETED, JobStatus.FAILED, JobStatus.CANCELLED
                    ]:
                        break

        except asyncio.CancelledError:
            logger.info(f"SSE connection cancelled for job {job_id}")
        except Exception as e:
            logger.error(f"Error in SSE stream for job {job_id}: {e}")
            raise

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",  # Disable nginx buffering
        },
    )


@router.get("/{job_id}/status", response_model=BatchJobStatus)
async def get_status(job_id: str):
    """
    Get current job status.

    Use this endpoint for polling-based progress tracking
    or as a fallback when SSE is not available.
    """
    job_manager = get_job_manager()
    job = job_manager.get_job_status(job_id)

    if not job:
        raise HTTPException(status_code=404, detail=f"Job not found: {job_id}")

    return job


@router.post("/{job_id}/cancel")
async def cancel_job(job_id: str):
    """
    Cancel a running job.

    Files that have already been processed will remain processed.
    Only pending files will be skipped.
    """
    job_manager = get_job_manager()

    # Check job exists
    job = job_manager.get_job_status(job_id)
    if not job:
        raise HTTPException(status_code=404, detail=f"Job not found: {job_id}")

    # Check if job can be cancelled
    if job.status in [JobStatus.COMPLETED, JobStatus.FAILED, JobStatus.CANCELLED]:
        raise HTTPException(
            status_code=400,
            detail=f"Cannot cancel job in '{job.status}' state"
        )

    # Request cancellation
    cancelled = job_manager.cancel_job(job_id)

    if cancelled:
        return {
            "job_id": job_id,
            "status": "cancellation_requested",
            "message": "Job cancellation has been requested. Processing will stop after current file."
        }
    else:
        raise HTTPException(
            status_code=500,
            detail="Failed to cancel job"
        )


@router.get("/jobs", response_model=List[BatchJobStatus])
async def list_jobs(limit: int = 20):
    """
    List recent batch jobs.

    Returns the most recent jobs ordered by creation time.
    """
    job_manager = get_job_manager()
    return job_manager.list_jobs(limit=limit)


@router.delete("/{job_id}")
async def delete_job(job_id: str):
    """
    Delete a job and its temporary files.

    Only completed, failed, or cancelled jobs can be deleted.
    """
    job_manager = get_job_manager()
    temp_storage = get_temp_storage_service()

    # Check job exists
    job = job_manager.get_job_status(job_id)
    if not job:
        raise HTTPException(status_code=404, detail=f"Job not found: {job_id}")

    # Check if job can be deleted
    if job.status not in [JobStatus.COMPLETED, JobStatus.FAILED, JobStatus.CANCELLED]:
        raise HTTPException(
            status_code=400,
            detail=f"Cannot delete job in '{job.status}' state. Cancel it first."
        )

    # Clean up temp files
    temp_storage.cancel_scheduled_cleanup(job_id)
    temp_storage.cleanup_job(job_id)

    # Clean up job resources
    job_manager.cleanup_job(job_id)

    return {
        "job_id": job_id,
        "status": "deleted",
        "message": "Job and temporary files have been deleted"
    }
