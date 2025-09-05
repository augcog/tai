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

    @staticmethod
    def get_problem_content_by_file_uuid_and_problem_id(
        db: Session, file_uuid: str, problem_id: str
    ) -> str:
        """
        Get the problem_index of a specific problem by file UUID and problem ID.
        Find all the problem content that share the same file UUID and problem_index.
        Concatenate the content of all problems with the same file UUID and problem ID by grouping problem ID to get first problem content.
        """
        try:
            problem = db.query(ProblemModel).filter(
                ProblemModel.file_uuid == file_uuid,
                ProblemModel.problem_id == problem_id
            ).first()
            if not problem:
                logger.warning(f"No problem found for file {file_uuid}, problem {problem_id}")
                return ""
            problem_index = problem.problem_index
            # Get all problems with the same file UUID and problem index
            problems = db.query(ProblemModel).filter(
                ProblemModel.file_uuid == file_uuid,
                ProblemModel.problem_index == problem_index
            ).all()
            # For each problem IDs, concatenate the content of problems
            id_set = set()
            problem_content = ""
            for p in problems:
                if p.problem_id not in id_set:
                    id_set.add(p.problem_id)
                    problem_content += f'Problem ID: {p.problem_id}\n'
                    problem_content += f'Problem Content: {p.problem_content}\n\n'
            problem_content+=f'User is working on problem {problem_id}'
            return problem_content


        except SQLAlchemyError as e:
            logger.error(f"Database error retrieving problem content for file {file_uuid}, problem {problem_id}: {str(e)}")
            return ""
        except Exception as e:
            logger.error(f"Unexpected error retrieving problem content for file {file_uuid}, problem {problem_id}: {str(e)}")
            return ""

