"""
Clean, unified file service
Best practices with easy user flow and auto-discovery
"""

import mimetypes
import os
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any, List, Set
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
            
        # Add path filtering for nested directory support
        if filters.get("path"):
            path = filters["path"].strip()
            if path:
                # Filter files that are in the specified path
                path_pattern = f"{path}/%"
                filter_conditions.append(FileModel.relative_path.like(path_pattern))

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

        file_path = self.base_dir / file_record.course_name / file_record.relative_path
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

    def browse_directory(self, db: Session, course_name: str, path: str = "") -> Dict[str, Any]:
        """
        Browse directory structure for a course with support for nested folders
        
        Args:
            db: Database session
            course_name: Course name (e.g., "ROAR Academy") 
            path: Directory path within the course (e.g., "Part One/practice")
        
        Returns:
            Dict with directories, files, breadcrumbs, and current_path
        """
        # Get all files for this course
        files_query = db.query(FileModel).filter(
            and_(
                FileModel.course_name == course_name,
                FileModel.relative_path.like('%/%')  # Valid relative paths
            )
        )
        
        all_files = files_query.all()
        
        # Parse directory structure from relative paths
        directories = self._extract_directories(all_files, path)
        
        # Get files in current directory only (not subdirectories)
        current_files = self._get_files_in_current_directory(all_files, path)
        
        # Build breadcrumbs
        breadcrumbs = self._build_breadcrumbs(path)
        
        return {
            "directories": directories,
            "files": current_files,
            "current_path": path,
            "breadcrumbs": breadcrumbs,
            "course_name": course_name
        }
    
    def _extract_directories(self, all_files: List[FileModel], current_path: str) -> List[Dict[str, Any]]:
        """Extract immediate subdirectories from file paths"""
        directory_paths: Set[str] = set()
        
        # Collect all unique directory paths
        for file in all_files:
            rel_path = file.relative_path
            
            # If we're at root, get top-level directories
            if not current_path:
                if '/' in rel_path:
                    # Get first directory component
                    first_dir = rel_path.split('/')[0]
                    directory_paths.add(first_dir)
            else:
                # Check if file is in a subdirectory of current_path
                if rel_path.startswith(current_path + '/'):
                    # Get remaining path after current_path
                    remaining = rel_path[len(current_path) + 1:]
                    if '/' in remaining:
                        # Get next directory component
                        next_dir = remaining.split('/')[0]
                        full_subdir_path = f"{current_path}/{next_dir}"
                        directory_paths.add(full_subdir_path)
        
        # Build directory info with file counts
        directories = []
        for dir_path in sorted(directory_paths):
            dir_name = dir_path.split('/')[-1]  # Get just the directory name
            
            # Count files in this directory and subdirectories
            file_count = sum(1 for f in all_files if f.relative_path.startswith(dir_path + '/'))
            
            # Check if this directory has subdirectories
            has_subdirs = any(
                f.relative_path.startswith(dir_path + '/') and 
                len(f.relative_path[len(dir_path) + 1:].split('/')) > 1
                for f in all_files
            )
            
            directories.append({
                "name": dir_name,
                "path": dir_path,
                "file_count": file_count,
                "has_subdirs": has_subdirs
            })
        
        return directories
    
    def _get_files_in_current_directory(self, all_files: List[FileModel], current_path: str) -> List[FileModel]:
        """Get files that are directly in the current directory (not in subdirectories)"""
        current_dir_files = []
        
        for file in all_files:
            rel_path = file.relative_path
            
            if not current_path:
                # At root - files directly in root (no subdirectory)
                if '/' in rel_path and rel_path.count('/') == 1:
                    current_dir_files.append(file)
            else:
                # In a specific directory - files directly in this directory
                if rel_path.startswith(current_path + '/'):
                    remaining = rel_path[len(current_path) + 1:]
                    # File is directly in current directory if no more slashes
                    if '/' not in remaining:
                        current_dir_files.append(file)
        
        return current_dir_files
    
    def _build_breadcrumbs(self, current_path: str) -> List[Dict[str, str]]:
        """Build breadcrumb navigation"""
        breadcrumbs = [{"name": "Root", "path": ""}]
        
        if current_path:
            path_parts = current_path.split('/')
            accumulated_path = ""
            
            for part in path_parts:
                if accumulated_path:
                    accumulated_path += f"/{part}"
                else:
                    accumulated_path = part
                    
                breadcrumbs.append({
                    "name": part,
                    "path": accumulated_path
                })
        
        return breadcrumbs

    @staticmethod
    def get_file_metadata_by_name(db: Session, file_name: str) -> Optional[FileModel]:
        """Get file metadata by file name"""
        return db.query(FileModel).filter(FileModel.file_name == file_name).first()


# Global service instance
file_service = FileService()
