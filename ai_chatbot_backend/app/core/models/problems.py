"""
Problems model for storing question data
"""

from sqlalchemy import Column, String, Integer, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid
import sqlalchemy
from app.core.database import PracticeBase

# SQLite fallback for JSON
if hasattr(sqlalchemy, 'JSON'):
    JSONType = sqlalchemy.JSON
else:
    JSONType = Text

class ProblemModel(PracticeBase):
    """
    Problem model for storing question data
    
    This table stores individual problems/questions that are associated
    with specific files through the file_uuid foreign key
    """
    __tablename__ = "problem"

    uuid = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()), index=True, comment="Unique UUID for each problem")
    file_uuid = Column(String(36), ForeignKey('file.uuid'), nullable=False, index=True, comment="Foreign key that maps back to file metadata")
    problem_index = Column(String(100), nullable=True, comment="Problem index within the file")
    problem_id = Column(String(100), nullable=True, comment="Problem identifier")
    problem_content = Column(String(1000), nullable=True, comment="Problem content")
    question_id = Column(Integer, nullable=True, comment="Question ID")
    question = Column(Text, nullable=False, comment="The question text")
    choices = Column(JSONType, nullable=False, comment="List of choice strings")
    answer = Column(JSONType, nullable=False, comment="Indices of correct answers (multi-select)")
    explanation = Column(Text, nullable=True, comment="Explanation of the answer")

    # Relationship to file
    file_metadata = relationship("FileMetadataModel", back_populates="problems")

    def __repr__(self):
        return f"<Problem(uuid={self.uuid}, question={self.question[:50]}...)>" 