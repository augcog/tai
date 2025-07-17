"""
Problem service for business logic operations
"""

from typing import List, Optional, Dict
from sqlalchemy.orm import Session
import uuid

from app.core.models.problems import ProblemModel
from app.schemas.problem import ProblemCreate, ProblemUpdate, ProblemListParams


class ProblemService:
    """Service class for problem management operations"""

    @staticmethod
    def get_problems_by_file_uuid(db: Session, file_uuid: str) -> List[ProblemModel]:
        """Get all problems for a specific file"""
        try:
            file_uuid_obj = uuid.UUID(file_uuid)
            return db.query(ProblemModel).filter(
                ProblemModel.file_uuid == file_uuid_obj
            ).all()
        except ValueError:
            return []

