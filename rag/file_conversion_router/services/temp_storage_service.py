"""Temporary storage service for batch file uploads.

This service manages temporary file storage for uploaded files,
including directory creation, file saving, and cleanup operations.
"""

import asyncio
import logging
import shutil
from pathlib import Path
from typing import List, Optional
from datetime import datetime, timedelta

from fastapi import UploadFile

from file_conversion_router.config import (
    BATCH_UPLOAD_CONFIG,
    get_temp_upload_dir,
)

logger = logging.getLogger(__name__)


class TempStorageService:
    """Service for managing temporary file storage during batch uploads."""

    def __init__(self):
        """Initialize the temp storage service."""
        self.base_dir = get_temp_upload_dir()
        self._cleanup_tasks: dict[str, asyncio.Task] = {}

    def create_job_directory(self, job_id: str) -> Path:
        """
        Create a directory for a batch job's files.

        Args:
            job_id: Unique identifier for the batch job

        Returns:
            Path to the created job directory
        """
        job_dir = self.base_dir / job_id
        job_dir.mkdir(parents=True, exist_ok=True)
        logger.info(f"Created job directory: {job_dir}")
        return job_dir

    async def save_uploaded_file(
        self,
        job_id: str,
        file: UploadFile,
        relative_path: Optional[str] = None,
    ) -> Path:
        """
        Save an uploaded file to the job's temporary directory.

        Args:
            job_id: Unique identifier for the batch job
            file: FastAPI UploadFile object
            relative_path: Optional relative path to preserve folder structure

        Returns:
            Path to the saved file
        """
        job_dir = self.base_dir / job_id

        # Determine file path
        if relative_path:
            # Preserve folder structure from upload
            file_path = job_dir / relative_path
            file_path.parent.mkdir(parents=True, exist_ok=True)
        else:
            file_path = job_dir / file.filename

        # Write file content
        content = await file.read()
        file_path.write_bytes(content)

        logger.debug(f"Saved file: {file_path} ({len(content)} bytes)")
        return file_path

    async def save_uploaded_files(
        self,
        job_id: str,
        files: List[UploadFile],
        preserve_paths: bool = False,
    ) -> List[Path]:
        """
        Save multiple uploaded files to the job's temporary directory.

        Args:
            job_id: Unique identifier for the batch job
            files: List of FastAPI UploadFile objects
            preserve_paths: Whether to preserve relative paths from filenames

        Returns:
            List of paths to saved files
        """
        self.create_job_directory(job_id)
        saved_paths = []

        for file in files:
            # Extract relative path from filename if present and preserve_paths is True
            if preserve_paths and "/" in file.filename:
                relative_path = file.filename
            else:
                relative_path = None

            path = await self.save_uploaded_file(job_id, file, relative_path)
            saved_paths.append(path)

        logger.info(f"Saved {len(saved_paths)} files for job {job_id}")
        return saved_paths

    def get_job_files(self, job_id: str) -> List[Path]:
        """
        Get all files in a job's directory.

        Args:
            job_id: Unique identifier for the batch job

        Returns:
            List of file paths in the job directory
        """
        job_dir = self.base_dir / job_id
        if not job_dir.exists():
            return []

        return [f for f in job_dir.rglob("*") if f.is_file()]

    def get_job_directory(self, job_id: str) -> Optional[Path]:
        """
        Get the directory for a job.

        Args:
            job_id: Unique identifier for the batch job

        Returns:
            Path to job directory if exists, None otherwise
        """
        job_dir = self.base_dir / job_id
        return job_dir if job_dir.exists() else None

    def cleanup_job(self, job_id: str) -> bool:
        """
        Remove a job's temporary directory and all its contents.

        Args:
            job_id: Unique identifier for the batch job

        Returns:
            True if cleanup was successful, False otherwise
        """
        job_dir = self.base_dir / job_id
        if not job_dir.exists():
            logger.debug(f"Job directory does not exist: {job_id}")
            return True

        try:
            shutil.rmtree(job_dir)
            logger.info(f"Cleaned up job directory: {job_id}")
            return True
        except Exception as e:
            logger.error(f"Failed to cleanup job {job_id}: {e}")
            return False

    async def schedule_cleanup(
        self,
        job_id: str,
        delay_seconds: Optional[int] = None,
    ) -> None:
        """
        Schedule cleanup of a job's directory after a delay.

        Args:
            job_id: Unique identifier for the batch job
            delay_seconds: Delay before cleanup (defaults to config value)
        """
        if delay_seconds is None:
            delay_seconds = BATCH_UPLOAD_CONFIG["cleanup_delay_seconds"]

        # Cancel existing cleanup task if any
        if job_id in self._cleanup_tasks:
            self._cleanup_tasks[job_id].cancel()

        async def delayed_cleanup():
            await asyncio.sleep(delay_seconds)
            self.cleanup_job(job_id)
            self._cleanup_tasks.pop(job_id, None)

        task = asyncio.create_task(delayed_cleanup())
        self._cleanup_tasks[job_id] = task
        logger.info(f"Scheduled cleanup for job {job_id} in {delay_seconds} seconds")

    def cancel_scheduled_cleanup(self, job_id: str) -> bool:
        """
        Cancel a scheduled cleanup for a job.

        Args:
            job_id: Unique identifier for the batch job

        Returns:
            True if a cleanup was cancelled, False if no cleanup was scheduled
        """
        if job_id in self._cleanup_tasks:
            self._cleanup_tasks[job_id].cancel()
            self._cleanup_tasks.pop(job_id, None)
            logger.info(f"Cancelled scheduled cleanup for job {job_id}")
            return True
        return False

    def cleanup_stale_jobs(self, max_age_hours: int = 24) -> int:
        """
        Clean up job directories older than the specified age.

        Args:
            max_age_hours: Maximum age in hours before cleanup

        Returns:
            Number of directories cleaned up
        """
        if not self.base_dir.exists():
            return 0

        cutoff_time = datetime.now() - timedelta(hours=max_age_hours)
        cleaned = 0

        for job_dir in self.base_dir.iterdir():
            if not job_dir.is_dir():
                continue

            # Check directory modification time
            mtime = datetime.fromtimestamp(job_dir.stat().st_mtime)
            if mtime < cutoff_time:
                try:
                    shutil.rmtree(job_dir)
                    cleaned += 1
                    logger.info(f"Cleaned stale job directory: {job_dir.name}")
                except Exception as e:
                    logger.error(f"Failed to clean stale directory {job_dir.name}: {e}")

        if cleaned > 0:
            logger.info(f"Cleaned up {cleaned} stale job directories")

        return cleaned

    def get_storage_stats(self) -> dict:
        """
        Get statistics about temporary storage usage.

        Returns:
            Dictionary with storage statistics
        """
        if not self.base_dir.exists():
            return {
                "total_jobs": 0,
                "total_files": 0,
                "total_size_bytes": 0,
            }

        total_jobs = 0
        total_files = 0
        total_size = 0

        for job_dir in self.base_dir.iterdir():
            if not job_dir.is_dir():
                continue

            total_jobs += 1
            for f in job_dir.rglob("*"):
                if f.is_file():
                    total_files += 1
                    total_size += f.stat().st_size

        return {
            "total_jobs": total_jobs,
            "total_files": total_files,
            "total_size_bytes": total_size,
            "total_size_mb": round(total_size / (1024 * 1024), 2),
        }


# Global singleton instance
_temp_storage_service: Optional[TempStorageService] = None


def get_temp_storage_service() -> TempStorageService:
    """Get the global temp storage service instance."""
    global _temp_storage_service
    if _temp_storage_service is None:
        _temp_storage_service = TempStorageService()
    return _temp_storage_service
