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
    def create_problem(db: Session, problem_data: ProblemCreate) -> ProblemModel:
        """Create new problem"""
        # Validate answer index
        if problem_data.answer >= len(problem_data.choices):
            raise ValueError("Answer index must be less than the number of choices")
        
        db_problem = ProblemModel(
            file_uuid=problem_data.file_uuid,
            problem_index=problem_data.problem_index,
            problem_id=problem_data.problem_id,
            problem_content=problem_data.problem_content,
            question_id=problem_data.question_id,
            question=problem_data.question,
            choices=problem_data.choices,
            answer=problem_data.answer,
            explanation=problem_data.explanation,
        )
        db.add(db_problem)
        db.commit()
        db.refresh(db_problem)
        return db_problem

    @staticmethod
    def get_problem_by_uuid(db: Session, problem_uuid: str) -> Optional[ProblemModel]:
        """Get problem by UUID"""
        try:
            uuid_obj = uuid.UUID(problem_uuid)
            return db.query(ProblemModel).filter(ProblemModel.uuid == uuid_obj).first()
        except ValueError:
            return None

    @staticmethod
    def update_problem(db: Session, problem_uuid: str, problem_data: ProblemUpdate) -> Optional[ProblemModel]:
        """Update problem information"""
        db_problem = ProblemService.get_problem_by_uuid(db, problem_uuid)
        if not db_problem:
            return None

        # Update only provided fields
        update_data = problem_data.dict(exclude_unset=True)
        
        # Validate answer index if choices are being updated
        if 'choices' in update_data and 'answer' in update_data:
            if update_data['answer'] >= len(update_data['choices']):
                raise ValueError("Answer index must be less than the number of choices")
        elif 'choices' in update_data:
            # If only choices are updated, validate against existing answer
            if db_problem.answer >= len(update_data['choices']):
                raise ValueError("Answer index must be less than the number of choices")
        
        for field, value in update_data.items():
            setattr(db_problem, field, value)

        db.commit()
        db.refresh(db_problem)
        return db_problem

    @staticmethod
    def delete_problem(db: Session, problem_uuid: str) -> bool:
        """Delete problem (hard delete)"""
        db_problem = ProblemService.get_problem_by_uuid(db, problem_uuid)
        if not db_problem:
            return False
        db.delete(db_problem)
        db.commit()
        return True

    @staticmethod
    def list_problems(db: Session, params: ProblemListParams) -> Dict:
        """List problems with filtering and pagination"""
        query = db.query(ProblemModel)

        # Apply filters
        if params.file_uuid:
            try:
                file_uuid_obj = uuid.UUID(params.file_uuid)
                query = query.filter(ProblemModel.file_uuid == file_uuid_obj)
            except ValueError:
                pass

        if params.search:
            search_term = f"%{params.search}%"
            query = query.filter(ProblemModel.question.ilike(search_term))

        # Get total count
        total_count = query.count()

        # Apply pagination
        offset = (params.page - 1) * params.limit
        problems = query.offset(offset).limit(params.limit).all()

        # Calculate pagination info
        has_next = offset + params.limit < total_count
        has_prev = params.page > 1

        return {
            "problems": problems,
            "total_count": total_count,
            "page": params.page,
            "limit": params.limit,
            "has_next": has_next,
            "has_prev": has_prev,
        }

    @staticmethod
    def get_problems_by_uuids(db: Session, problem_uuids: List[str]) -> List[ProblemModel]:
        """Get multiple problems by their UUIDs"""
        try:
            uuid_objects = [uuid.UUID(uuid_str) for uuid_str in problem_uuids]
            return db.query(ProblemModel).filter(ProblemModel.uuid.in_(uuid_objects)).all()
        except ValueError:
            return []

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

    @staticmethod
    def validate_answer(problem_uuid: str, user_answer: int, db: Session) -> Dict:
        """Validate a user's answer to a problem"""
        problem = ProblemService.get_problem_by_uuid(db, problem_uuid)
        if not problem:
            return {"error": "Problem not found"}
        
        if user_answer >= len(problem.choices):
            return {"error": "Invalid answer index"}
        
        is_correct = user_answer == problem.answer
        
        return {
            "is_correct": is_correct,
            "correct_answer": problem.answer,
            "explanation": problem.explanation,
            "choices": problem.choices
        } 