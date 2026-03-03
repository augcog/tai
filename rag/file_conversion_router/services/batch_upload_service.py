"""Batch upload service for orchestrating file conversion jobs.

This service manages batch conversion jobs including:
- Job creation and status tracking
- File validation
- Batch processing with progress callbacks
"""

import asyncio
import logging
import uuid
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Callable, Any

from fastapi import UploadFile

from file_conversion_router.config import (
    BATCH_UPLOAD_CONFIG,
    get_allowed_extensions,
    get_max_file_size_bytes,
    get_max_total_size_bytes,
    get_course_db_path,
    get_course_output_dir,
)
from file_conversion_router.web.schemas import (
    JobStatus,
    FileStatus,
    BatchJobStatus,
    FileError,
    FileResult,
    BatchConversionResult,
    ValidationError,
    BatchValidationResult,
    ProgressEvent,
    ProgressEventType,
)
from file_conversion_router.services.temp_storage_service import get_temp_storage_service

logger = logging.getLogger(__name__)


class FileValidator:
    """Validates uploaded files before processing."""

    def __init__(self):
        self.allowed_extensions = get_allowed_extensions()
        self.max_file_size = get_max_file_size_bytes()
        self.max_total_size = get_max_total_size_bytes()
        self.max_files = BATCH_UPLOAD_CONFIG["max_files_per_batch"]

    def validate_extension(self, filename: str) -> Optional[str]:
        """
        Validate file extension.

        Returns None if valid, error message if invalid.
        """
        ext = Path(filename).suffix.lower()
        if ext not in self.allowed_extensions:
            return f"Unsupported file type: {ext}. Allowed: {self.allowed_extensions}"
        return None

    def validate_size(self, file_size: int, filename: str) -> Optional[str]:
        """
        Validate file size.

        Returns None if valid, error message if invalid.
        """
        if file_size > self.max_file_size:
            max_mb = self.max_file_size / (1024 * 1024)
            file_mb = file_size / (1024 * 1024)
            return f"File too large: {file_mb:.1f}MB exceeds {max_mb:.0f}MB limit"
        return None

    def validate_batch(self, files: List[UploadFile]) -> BatchValidationResult:
        """
        Validate a batch of files.

        Returns BatchValidationResult with valid and invalid files.
        """
        valid_files = []
        invalid_files = []
        total_size = 0

        # Check file count
        if len(files) > self.max_files:
            # All files beyond limit are invalid
            for file in files[self.max_files:]:
                invalid_files.append(ValidationError(
                    file_name=file.filename,
                    error_type="batch_limit",
                    message=f"Exceeds max files per batch ({self.max_files})"
                ))
            files = files[:self.max_files]

        for file in files:
            # Check extension
            ext_error = self.validate_extension(file.filename)
            if ext_error:
                invalid_files.append(ValidationError(
                    file_name=file.filename,
                    error_type="invalid_extension",
                    message=ext_error
                ))
                continue

            # Check size (if available)
            if file.size is not None:
                size_error = self.validate_size(file.size, file.filename)
                if size_error:
                    invalid_files.append(ValidationError(
                        file_name=file.filename,
                        error_type="file_too_large",
                        message=size_error
                    ))
                    continue

                # Check total size
                if total_size + file.size > self.max_total_size:
                    invalid_files.append(ValidationError(
                        file_name=file.filename,
                        error_type="total_size_exceeded",
                        message=f"Total upload size would exceed {self.max_total_size / (1024*1024):.0f}MB limit"
                    ))
                    continue

                total_size += file.size

            valid_files.append(file.filename)

        return BatchValidationResult(
            valid_files=valid_files,
            invalid_files=invalid_files,
            total_size_bytes=total_size,
            is_valid=len(invalid_files) == 0
        )


class BatchJobManager:
    """Manages batch conversion job lifecycle and status tracking."""

    def __init__(self):
        self._jobs: Dict[str, BatchJobStatus] = {}
        self._progress_queues: Dict[str, asyncio.Queue] = {}
        self._cancel_events: Dict[str, asyncio.Event] = {}

    def create_job(
        self,
        course_code: str,
        course_name: str,
        file_count: int,
    ) -> str:
        """
        Create a new batch job.

        Returns the job ID.
        """
        job_id = str(uuid.uuid4())
        now = datetime.now()

        self._jobs[job_id] = BatchJobStatus(
            job_id=job_id,
            status=JobStatus.PENDING,
            total_files=file_count,
            processed_files=0,
            successful_files=0,
            failed_files=0,
            skipped_files=0,
            current_file=None,
            errors=[],
            results=[],
            created_at=now,
            updated_at=now,
        )

        # Create progress queue and cancel event
        self._progress_queues[job_id] = asyncio.Queue()
        self._cancel_events[job_id] = asyncio.Event()

        logger.info(f"Created job {job_id} for {file_count} files")
        return job_id

    def get_job_status(self, job_id: str) -> Optional[BatchJobStatus]:
        """Get the current status of a job."""
        return self._jobs.get(job_id)

    def update_job_status(
        self,
        job_id: str,
        status: Optional[JobStatus] = None,
        current_file: Optional[str] = None,
        increment_processed: bool = False,
        increment_successful: bool = False,
        increment_failed: bool = False,
        increment_skipped: bool = False,
        add_error: Optional[FileError] = None,
        add_result: Optional[FileResult] = None,
    ) -> Optional[BatchJobStatus]:
        """Update job status and return the updated status."""
        job = self._jobs.get(job_id)
        if not job:
            return None

        if status:
            job.status = status
        if current_file is not None:
            job.current_file = current_file
        if increment_processed:
            job.processed_files += 1
        if increment_successful:
            job.successful_files += 1
        if increment_failed:
            job.failed_files += 1
        if increment_skipped:
            job.skipped_files += 1
        if add_error:
            job.errors.append(add_error)
        if add_result:
            job.results.append(add_result)

        job.updated_at = datetime.now()

        if status in [JobStatus.COMPLETED, JobStatus.FAILED, JobStatus.CANCELLED]:
            job.completed_at = datetime.now()
            job.current_file = None

        return job

    def get_progress_queue(self, job_id: str) -> Optional[asyncio.Queue]:
        """Get the progress queue for a job."""
        return self._progress_queues.get(job_id)

    def get_cancel_event(self, job_id: str) -> Optional[asyncio.Event]:
        """Get the cancel event for a job."""
        return self._cancel_events.get(job_id)

    def cancel_job(self, job_id: str) -> bool:
        """
        Request cancellation of a job.

        Returns True if job was found and cancellation requested.
        """
        cancel_event = self._cancel_events.get(job_id)
        if cancel_event:
            cancel_event.set()
            self.update_job_status(job_id, status=JobStatus.CANCELLED)
            logger.info(f"Cancellation requested for job {job_id}")
            return True
        return False

    def is_cancelled(self, job_id: str) -> bool:
        """Check if a job has been cancelled."""
        cancel_event = self._cancel_events.get(job_id)
        return cancel_event.is_set() if cancel_event else False

    async def push_progress(self, job_id: str, event: ProgressEvent) -> None:
        """Push a progress event to the job's queue."""
        queue = self._progress_queues.get(job_id)
        if queue:
            await queue.put(event)

    def cleanup_job(self, job_id: str) -> None:
        """Clean up job resources (queues, events)."""
        self._progress_queues.pop(job_id, None)
        self._cancel_events.pop(job_id, None)
        # Don't remove from _jobs - keep for status queries

    def list_jobs(self, limit: int = 100) -> List[BatchJobStatus]:
        """List recent jobs."""
        jobs = sorted(
            self._jobs.values(),
            key=lambda j: j.created_at,
            reverse=True
        )
        return jobs[:limit]


class BatchProcessor:
    """Processes batch file conversions."""

    def __init__(self, job_manager: BatchJobManager):
        self.job_manager = job_manager

    async def process_batch(
        self,
        job_id: str,
        file_paths: List[Path],
        course_code: str,
        course_name: str,
        output_dir: Path,
        db_path: Path,
        auto_embed: bool = True,
    ) -> BatchConversionResult:
        """
        Process a batch of files.

        This runs the conversion and reports progress through the job manager.
        """
        from file_conversion_router.api import batch_convert_files

        start_time = datetime.now()

        # Update job status to processing
        self.job_manager.update_job_status(job_id, status=JobStatus.PROCESSING)

        # Push job start event
        await self.job_manager.push_progress(job_id, ProgressEvent(
            event_type=ProgressEventType.JOB_START,
            job_id=job_id,
            total_files=len(file_paths),
        ))

        # Progress callback that updates job status and pushes events
        async def on_progress(file_name: str, status: str, error: Optional[str]):
            # Check for cancellation
            if self.job_manager.is_cancelled(job_id):
                raise asyncio.CancelledError("Job cancelled by user")

            if status == "started":
                self.job_manager.update_job_status(job_id, current_file=file_name)
                await self.job_manager.push_progress(job_id, ProgressEvent(
                    event_type=ProgressEventType.FILE_START,
                    job_id=job_id,
                    file_name=file_name,
                ))

            elif status == "completed":
                self.job_manager.update_job_status(
                    job_id,
                    increment_processed=True,
                    increment_successful=True,
                    add_result=FileResult(
                        file_name=file_name,
                        status=FileStatus.COMPLETED,
                    ),
                )
                await self.job_manager.push_progress(job_id, ProgressEvent(
                    event_type=ProgressEventType.FILE_DONE,
                    job_id=job_id,
                    file_name=file_name,
                    file_status=FileStatus.COMPLETED,
                ))

            elif status == "failed":
                self.job_manager.update_job_status(
                    job_id,
                    increment_processed=True,
                    increment_failed=True,
                    add_error=FileError(file_name=file_name, error_message=error or "Unknown error"),
                    add_result=FileResult(
                        file_name=file_name,
                        status=FileStatus.FAILED,
                        error=error,
                    ),
                )
                await self.job_manager.push_progress(job_id, ProgressEvent(
                    event_type=ProgressEventType.FILE_ERROR,
                    job_id=job_id,
                    file_name=file_name,
                    file_status=FileStatus.FAILED,
                    error_message=error,
                ))

            elif status == "skipped":
                self.job_manager.update_job_status(
                    job_id,
                    increment_processed=True,
                    increment_skipped=True,
                    add_result=FileResult(
                        file_name=file_name,
                        status=FileStatus.SKIPPED,
                    ),
                )
                await self.job_manager.push_progress(job_id, ProgressEvent(
                    event_type=ProgressEventType.FILE_DONE,
                    job_id=job_id,
                    file_name=file_name,
                    file_status=FileStatus.SKIPPED,
                ))

        # Synchronous callback wrapper for batch_convert_files
        def sync_progress_callback(file_name: str, status: str, error: Optional[str]):
            # Schedule the async callback
            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    asyncio.create_task(on_progress(file_name, status, error))
                else:
                    loop.run_until_complete(on_progress(file_name, status, error))
            except Exception as e:
                logger.error(f"Error in progress callback: {e}")

        try:
            # Run the batch conversion
            result = batch_convert_files(
                files=file_paths,
                course_code=course_code,
                course_name=course_name,
                output_dir=output_dir,
                db_path=db_path,
                auto_embed=auto_embed,
                progress_callback=sync_progress_callback,
            )

            # Calculate duration
            duration = (datetime.now() - start_time).total_seconds()

            # Build final result
            final_status = self.job_manager.get_job_status(job_id)
            conversion_result = BatchConversionResult(
                job_id=job_id,
                status=JobStatus.COMPLETED,
                files_processed=result["files_processed"],
                files_failed=result["files_failed"],
                files_skipped=result["files_skipped"],
                total_chunks=result["total_chunks"],
                errors=[FileError(**e) for e in result["errors"]],
                results=[FileResult(**r) for r in result["results"]],
                duration_seconds=duration,
                embedding_error=result.get("embedding_error"),
            )

            # Update job to completed
            self.job_manager.update_job_status(job_id, status=JobStatus.COMPLETED)

            # Push completion event
            await self.job_manager.push_progress(job_id, ProgressEvent(
                event_type=ProgressEventType.JOB_COMPLETE,
                job_id=job_id,
                total_files=len(file_paths),
                processed_files=result["files_processed"] + result["files_failed"] + result["files_skipped"],
                successful_files=result["files_processed"],
                failed_files=result["files_failed"],
                result=conversion_result,
            ))

            return conversion_result

        except asyncio.CancelledError:
            duration = (datetime.now() - start_time).total_seconds()
            final_status = self.job_manager.get_job_status(job_id)

            conversion_result = BatchConversionResult(
                job_id=job_id,
                status=JobStatus.CANCELLED,
                files_processed=final_status.successful_files if final_status else 0,
                files_failed=final_status.failed_files if final_status else 0,
                files_skipped=final_status.skipped_files if final_status else 0,
                total_chunks=0,
                errors=final_status.errors if final_status else [],
                results=final_status.results if final_status else [],
                duration_seconds=duration,
            )

            await self.job_manager.push_progress(job_id, ProgressEvent(
                event_type=ProgressEventType.JOB_COMPLETE,
                job_id=job_id,
                result=conversion_result,
            ))

            return conversion_result

        except Exception as e:
            logger.error(f"Batch processing failed for job {job_id}: {e}")
            duration = (datetime.now() - start_time).total_seconds()

            self.job_manager.update_job_status(job_id, status=JobStatus.FAILED)

            conversion_result = BatchConversionResult(
                job_id=job_id,
                status=JobStatus.FAILED,
                files_processed=0,
                files_failed=len(file_paths),
                files_skipped=0,
                total_chunks=0,
                errors=[FileError(file_name="batch", error_message=str(e))],
                results=[],
                duration_seconds=duration,
            )

            await self.job_manager.push_progress(job_id, ProgressEvent(
                event_type=ProgressEventType.JOB_COMPLETE,
                job_id=job_id,
                result=conversion_result,
            ))

            return conversion_result


# Global singleton instances
_job_manager: Optional[BatchJobManager] = None
_batch_processor: Optional[BatchProcessor] = None
_file_validator: Optional[FileValidator] = None


def get_job_manager() -> BatchJobManager:
    """Get the global job manager instance."""
    global _job_manager
    if _job_manager is None:
        _job_manager = BatchJobManager()
    return _job_manager


def get_batch_processor() -> BatchProcessor:
    """Get the global batch processor instance."""
    global _batch_processor
    if _batch_processor is None:
        _batch_processor = BatchProcessor(get_job_manager())
    return _batch_processor


def get_file_validator() -> FileValidator:
    """Get the global file validator instance."""
    global _file_validator
    if _file_validator is None:
        _file_validator = FileValidator()
    return _file_validator
