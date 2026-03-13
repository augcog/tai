"""
Module service for managing course modules via the module table
"""

from typing import Dict, List, Optional
from sqlalchemy.orm import Session
from sqlalchemy import and_

from app.core.models.metadata import FileModel, ModuleModel


class ModuleService:
    """Service for querying course modules from the database."""

    def list_modules(self, db: Session, course_code: str) -> List[ModuleModel]:
        """Return all modules for a course, ordered by path."""
        return (
            db.query(ModuleModel)
            .filter(ModuleModel.course_code == course_code)
            .order_by(ModuleModel.path)
            .all()
        )

    def get_by_uuid(self, db: Session, module_uuid: str) -> Optional[ModuleModel]:
        """Return a module by its UUID, or None if not found."""
        return db.query(ModuleModel).filter(ModuleModel.uuid == module_uuid).first()

    def get_module_files(
        self, db: Session, module_uuid: str
    ) -> List[FileModel]:
        """
        Return all files within the module identified by module_uuid.

        Raises:
            ValueError: If no module with the given UUID exists.
        """
        module = self.get_by_uuid(db, module_uuid)
        if not module:
            raise ValueError(f"Module not found: {module_uuid}")

        # module.path already includes the course directory prefix
        # e.g. "CS 61A/practice/hw/hw01" — so match relative_path directly
        return (
            db.query(FileModel)
            .filter(FileModel.relative_path.like(f"{module.path}/%"))
            .all()
        )

    def get_module_path(self, db: Session, module_uuid: str) -> str:
        """
        Resolve a module UUID to its path string for RAG filtering.

        Raises:
            ValueError: If no module with the given UUID exists.
        """
        module = self.get_by_uuid(db, module_uuid)
        if not module:
            raise ValueError(f"Module not found: {module_uuid}")
        return module.path


# Global service instance
module_service = ModuleService()
