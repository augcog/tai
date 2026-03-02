"""
Module service for extracting and managing course modules
"""

import re
from typing import Dict, List, Set, Tuple
from sqlalchemy.orm import Session
from sqlalchemy import and_, func

from app.core.models.metadata import FileModel
from app.utils.path_validation import is_valid_module_path


class ModuleService:
    """
    Service for managing course modules.

    A module is defined as:
    - Subdirectories under practice/*/ (grandchildren of practice/)
    - Subdirectories under study/ (children of study/)
    - Subdirectories under support/ (children of support/)
    """

    @staticmethod
    def _categorize_path(path: str) -> Tuple[str, str, bool]:
        """
        Categorize a file path and determine if it represents a module.

        Returns:
            Tuple of (category, module_path, is_module)
            - category: 'practice', 'study', 'support', or None
            - module_path: The path to the module directory
            - is_module: Whether this path is within a module
        """
        parts = path.split('/')

        # Check for practice/*/ pattern (grandchildren)
        if len(parts) >= 3 and parts[0] == 'practice':
            # practice/labs/lab01/file.pdf -> module is "practice/labs/lab01"
            module_path = '/'.join(parts[:3])
            return 'practice', module_path, True

        # Check for study/ pattern (children)
        if len(parts) >= 2 and parts[0] == 'study':
            # study/week1/file.pdf -> module is "study/week1"
            module_path = '/'.join(parts[:2])
            return 'study', module_path, True

        # Check for support/ pattern (children)
        if len(parts) >= 2 and parts[0] == 'support':
            # support/resources/file.pdf -> module is "support/resources"
            module_path = '/'.join(parts[:2])
            return 'support', module_path, True

        return None, None, False

    def list_modules(self, db: Session, course_code: str) -> List[Dict]:
        """
        List all modules for a given course.

        Args:
            db: Database session
            course_code: Course code (e.g., "CS61A")

        Returns:
            List of module dictionaries with name, path, category, and file_count
        """
        # Get all files for this course
        files = db.query(FileModel).filter(
            and_(
                FileModel.course_code == course_code,
                FileModel.relative_path.like('%/%')  # Valid relative paths
            )
        ).all()

        # Extract modules from file paths
        modules: Dict[str, Dict] = {}

        for file in files:
            category, module_path, is_module = self._categorize_path(file.relative_path)

            if is_module and module_path:
                if module_path not in modules:
                    # Extract module name (last part of path)
                    module_name = module_path.split('/')[-1]
                    modules[module_path] = {
                        'name': module_name,
                        'path': module_path,
                        'category': category,
                        'file_count': 0,
                        'course_code': course_code
                    }
                modules[module_path]['file_count'] += 1

        # Convert to sorted list
        module_list = sorted(modules.values(), key=lambda x: x['path'])

        return module_list

    def get_module_files(
        self, db: Session, course_code: str, module_path: str
    ) -> List[FileModel]:
        """
        Get all files within a specific module.

        Args:
            db: Database session
            course_code: Course code (e.g., "CS61A")
            module_path: Module path (e.g., "practice/labs/lab01" or "study/week1")

        Returns:
            List of FileModel objects

        Raises:
            ValueError: If module_path is not a valid module structure
        """
        # Validate that module_path is a valid module
        if not is_valid_module_path(module_path):
            raise ValueError(
                f"Invalid module path: {module_path}. "
                "Module paths must follow the structure: "
                "practice/*/<name>, study/<name>, or support/<name>"
            )

        # Files in this module start with the module path
        pattern = f"{module_path}/%"

        files = db.query(FileModel).filter(
            and_(
                FileModel.course_code == course_code,
                FileModel.relative_path.like(pattern)
            )
        ).all()

        return files

    @staticmethod
    def extract_module_from_path(path: str) -> str:
        """
        Extract module path from a file path.

        Args:
            path: File path (e.g., "practice/labs/lab01/file.pdf")

        Returns:
            Module path (e.g., "practice/labs/lab01") or empty string if not in a module
        """
        service = ModuleService()
        category, module_path, is_module = service._categorize_path(path)
        return module_path if is_module else ""


# Global service instance
module_service = ModuleService()
