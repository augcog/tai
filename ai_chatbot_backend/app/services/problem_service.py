"""
Problem service for business logic operations
"""

from typing import List
from sqlalchemy.orm import Session

from app.core.models.metadata import ProblemModel


class ProblemService:
    """Service class for problem management operations"""

    @staticmethod
    def get_problems_by_file_uuid(db: Session, file_uuid: str) -> List[ProblemModel]:
        """Get all problems for a specific file"""
        try:
            return db.query(ProblemModel).filter(
                ProblemModel.file_uuid == file_uuid
            ).all()
        except ValueError:
            return []

