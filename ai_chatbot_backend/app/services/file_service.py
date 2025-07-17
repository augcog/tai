"""
Clean, unified file service
Best practices with easy user flow and auto-discovery
"""

import mimetypes
import os
import re
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any
from uuid import UUID, uuid4

from fastapi import HTTPException, status
from fastapi.responses import FileResponse
from sqlalchemy.orm import Session
from sqlalchemy import and_

from app.config import settings
from app.core.models.files import FileRegistry


class FileService:
    """
    Unified file service with clean design and best practices

    Features:
    - Auto-discovery: Files found automatically, no manual rescans
    - UUID-based: Secure access without exposing file paths
    - Clean API: Simple, intuitive interface
    - Best practices: Proper error handling, validation, security
    """

    def __init__(self):
        self.base_dir = self._get_base_directory()
        self._cache = {}  # Simple cache for performance
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

    def _extract_metadata(self, relative_path: str) -> Dict[str, Any]:
        """Extract simple, essential metadata from file path"""
        parts = relative_path.split("/")
        metadata = {}

        # Extract course code (first directory)
        if len(parts) >= 1:
            metadata["course_code"] = parts[0]

        # Extract category (second directory) - simplified
        if len(parts) >= 2:
            category_map = {
                "documents": "document",
                "videos": "video",
                "audios": "audio",
                "others": "other",
            }
            metadata["category"] = category_map.get(parts[1], "other")

        # Generate clean title from filename
        filename = parts[-1] if parts else ""
        if filename:
            title = filename.rsplit(".", 1)[0]  # Remove extension
            title = re.sub(r"[_-]", " ", title)  # Replace with spaces
            title = re.sub(r"\s+", " ", title).strip()  # Normalize
            title = title.title()  # Title case
            metadata["title"] = title

        return metadata

    def _discover_and_register_file(
        self, db: Session, file_path: Path, relative_path: str
    ) -> Optional[FileRegistry]:
        """Discover and register a single file with proper error handling"""
        try:
            # Check if already registered
            existing = (
                db.query(FileRegistry)
                .filter(FileRegistry.relative_path == relative_path)
                .first()
            )

            # Get file stats
            file_stat = file_path.stat()
            mime_type, _ = mimetypes.guess_type(str(file_path))
            if not mime_type:
                mime_type = "application/octet-stream"

            if existing:
                # Update if file changed
                if existing.size_bytes != file_stat.st_size:
                    existing.size_bytes = file_stat.st_size
                    existing.mime_type = mime_type
                    existing.modified_at = datetime.now()
                    db.commit()

                return existing
            else:
                # Create new record
                metadata = self._extract_metadata(relative_path)

                file_record = FileRegistry(
                    id=uuid4(),
                    file_name=file_path.name,
                    relative_path=relative_path,
                    mime_type=mime_type,
                    size_bytes=file_stat.st_size,
                    **metadata,
                )

                db.add(file_record)
                db.commit()
                return file_record

        except Exception as e:
            print(f"Error registering file {file_path}: {e}")
            return None

    def _auto_discover_files(self, db: Session, limit: int = 50):
        """Auto-discover new files efficiently"""
        if not self.base_dir.exists():
            return

        try:
            discovered = 0
            for file_path in self.base_dir.rglob("*"):
                if discovered >= limit:  # Limit to keep API responsive
                    break

                if not file_path.is_file():
                    continue

                # Skip hidden files and system files
                if any(part.startswith(".") for part in file_path.parts):
                    continue

                relative_path = str(file_path.relative_to(self.base_dir))

                # Check cache to avoid repeated processing
                if relative_path in self._cache:
                    continue

                # Check if already in database
                exists = (
                    db.query(FileRegistry)
                    .filter(FileRegistry.relative_path == relative_path)
                    .first()
                )

                if not exists:
                    self._discover_and_register_file(db, file_path, relative_path)
                    discovered += 1

                # Cache to avoid repeated checks
                self._cache[relative_path] = True

        except Exception as e:
            print(f"Auto-discovery error: {e}")

    def list_files(self, db: Session, **filters) -> Dict[str, Any]:
        """
        List files with auto-discovery and comprehensive filtering

        Returns both files and pagination info in a clean format
        """
        # Auto-discover new files first
        self._auto_discover_files(db)

        # Build query
        query = db.query(FileRegistry)

        # Apply simple, essential filters only
        filter_conditions = []
        if filters.get("course_code"):
            filter_conditions.append(FileRegistry.course_code == filters["course_code"])

        if filters.get("category"):
            filter_conditions.append(FileRegistry.category == filters["category"])

        if filters.get("search"):
            search_term = f"%{filters['search']}%"
            filter_conditions.append(
                FileRegistry.file_name.ilike(search_term)
                | FileRegistry.title.ilike(search_term)
            )

        # Apply all filters
        if filter_conditions:
            query = query.filter(and_(*filter_conditions))

        # Get total count
        total_count = query.count()

        # Apply sorting
        sort_by = filters.get("sort_by", "created_at")
        sort_order = filters.get("sort_order", "desc")

        sort_column = getattr(FileRegistry, sort_by, FileRegistry.created_at)
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

    def get_file_by_id(self, db: Session, file_id: UUID) -> Optional[FileRegistry]:
        """Get file by UUID with validation"""
        return (
            db.query(FileRegistry)
            .filter(and_(FileRegistry.id == file_id))
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

        return FileResponse(
            path=str(file_path),
            media_type=file_record.mime_type,
            filename=file_record.file_name,
        )

    def get_stats(self, db: Session) -> Dict[str, Any]:
        """Get comprehensive file system statistics"""
        # Auto-discover before stats
        self._auto_discover_files(db)

        total_files = (
            db.query(FileRegistry).count()
        )

        # Get course breakdown
        from sqlalchemy import func

        course_stats = (
            db.query(
                FileRegistry.course_code, func.count(FileRegistry.id).label("count")
            )
            .filter(
                FileRegistry.course_code.isnot(None)
            )
            .group_by(FileRegistry.course_code)
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
    def get_file_metadata_by_name(db: Session, file_name: str) -> Optional[FileMetadataModel]:
        """Get file metadata by file name"""
        return db.query(FileMetadataModel).filter(FileMetadataModel.file_name == file_name).first()


# Global service instance
file_service = FileService()
