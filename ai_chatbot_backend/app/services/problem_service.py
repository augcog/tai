"""
Problem service for business logic operations
"""

from typing import List
from sqlalchemy.orm import Session
from sqlalchemy.exc import SQLAlchemyError
import logging

from app.core.models.metadata import ProblemModel

logger = logging.getLogger(__name__)


class ProblemService:
    """Service class for problem management operations"""

    @staticmethod
    def get_problems_by_file_uuid(db: Session, file_uuid: str) -> List[ProblemModel]:
        """Get all problems for a specific file"""
        try:
            return db.query(ProblemModel).filter(
                ProblemModel.file_uuid == file_uuid
            ).all()
        except SQLAlchemyError as e:
            logger.error(f"Database error retrieving problems for file {file_uuid}: {str(e)}")
            return []
        except Exception as e:
            logger.error(f"Unexpected error retrieving problems for file {file_uuid}: {str(e)}")
            return []

