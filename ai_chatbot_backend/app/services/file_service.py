"""
Clean, unified file service
Best practices with easy user flow and auto-discovery
"""

import mimetypes
import os
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any
from uuid import UUID

from fastapi import HTTPException, status
from fastapi.responses import FileResponse
from sqlalchemy.orm import Session
from sqlalchemy import and_

from app.config import settings
from app.core.models.metadata import FileModel


class FileService:
    """
    Unified file service with clean design and best practices

    Features:
    - MongoDB-based: Files loaded from MongoDB database only
    - UUID-based: Secure access without exposing file paths
    - Clean API: Simple, intuitive interface
    - Best practices: Proper error handling, validation, security
    """

    def __init__(self):
        self.base_dir = self._get_base_directory()
        mimetypes.init()

    def _get_base_directory(self) -> Path:
        """Get base directory with proper validation"""
        if hasattr(settings, "DATA_DIR") and settings.DATA_DIR:
            data_dir = str(settings.DATA_DIR).split("#")[0].strip().strip("\"'")
            if os.path.isabs(data_dir):
                base_path = Path(data_dir)
            else:
                project_root = Path(__file__).parent.parent.parent
                base_path = project_root / data_dir
        else:
            project_root = Path(__file__).parent.parent.parent
            base_path = project_root / "data"

        # Ensure directory exists
        base_path.mkdir(parents=True, exist_ok=True)
        return base_path




    def list_files(self, db: Session, **filters) -> Dict[str, Any]:
        """
        List files with comprehensive filtering

        Returns both files and pagination info in a clean format
        """
        # Removed auto-discovery - only rely on MongoDB data

        # Build query
        query = db.query(FileModel)

        # Apply simple, essential filters only
        filter_conditions = []
        
        # Only include files with valid relative paths
        # Format should be: COURSE/file.ext or COURSE/subfolder/file.ext
        filter_conditions.append(FileModel.relative_path.like('%/%'))
        
        if filters.get("course_code"):
            filter_conditions.append(FileModel.course_code == filters["course_code"])

        if filters.get("search"):
            search_term = f"%{filters['search']}%"
            filter_conditions.append(
                FileModel.file_name.ilike(search_term)
            )

        # Apply all filters
        if filter_conditions:
            query = query.filter(and_(*filter_conditions))

        # Get total count
        total_count = query.count()

        # Apply sorting by file_name for now (metadata.db doesn't have created_at)
        sort_by = filters.get("sort_by", "file_name")
        sort_order = filters.get("sort_order", "asc")

        sort_column = getattr(FileModel, sort_by, FileModel.file_name)
        if sort_order.lower() == "asc":
            query = query.order_by(sort_column.asc())
        else:
            query = query.order_by(sort_column.desc())

        # Apply pagination
        page = filters.get("page", 1)
        limit = filters.get("limit", 100)
        offset = (page - 1) * limit

        files = query.offset(offset).limit(limit).all()

        return {
            "files": files,
            "total_count": total_count,
            "page": page,
            "limit": limit,
            "has_next": offset + limit < total_count,
            "has_prev": page > 1,
        }

    def get_file_by_id(self, db: Session, file_id: UUID) -> Optional[FileModel]:
        """Get file by UUID with validation"""
        return (
            db.query(FileModel)
            .filter(FileModel.uuid == str(file_id))
            .first()
        )

    def get_file_content(self, db: Session, file_id: UUID) -> FileResponse:
        """Get file content with security and access tracking"""
        file_record = self.get_file_by_id(db, file_id)
        if not file_record:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail="File not found"
            )

        file_path = self.base_dir / file_record.relative_path
        if not file_path.exists():
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail="File not found on disk"
            )

        # Security check - ensure file is within base directory
        try:
            file_path.resolve().relative_to(self.base_dir.resolve())
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN, detail="Access denied"
            )

        # Note: Access tracking removed for simplicity
        # Could be added back if needed

        # Get mime type since it's not in metadata db
        mime_type, _ = mimetypes.guess_type(str(file_path))
        if not mime_type:
            mime_type = "application/octet-stream"

        return FileResponse(
            path=str(file_path),
            media_type=mime_type,
            filename=file_record.file_name,
        )

    def get_stats(self, db: Session) -> Dict[str, Any]:
        """Get comprehensive file system statistics"""
        # Auto-discover before stats
        self._auto_discover_files(db)

        total_files = db.query(FileModel).count()

        # Get course breakdown
        from sqlalchemy import func

        course_stats = (
            db.query(
                FileModel.course_code, func.count(FileModel.uuid).label("count")
            )
            .filter(FileModel.course_code.isnot(None))
            .group_by(FileModel.course_code)
            .all()
        )

        return {
            "total_files": total_files,
            "base_directory": str(self.base_dir),
            "auto_discovery": "enabled",
            "courses": {code: count for code, count in course_stats},
            "last_updated": datetime.now(),
        }

    @staticmethod
    def get_file_metadata_by_name(db: Session, file_name: str) -> Optional[FileModel]:
        """Get file metadata by file name"""
        return db.query(FileModel).filter(FileModel.file_name == file_name).first()


# Global service instance
file_service = FileService()
