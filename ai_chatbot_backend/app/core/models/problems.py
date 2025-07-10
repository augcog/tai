"""
Problems model for storing question data
"""

from sqlalchemy import Column, String, Integer, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid
import sqlalchemy
from app.core.database import Base

# SQLite fallback for JSON
if hasattr(sqlalchemy, 'JSON'):
    JSONType = sqlalchemy.JSON
else:
    JSONType = Text

class ProblemModel(Base):
    """
    Problem model for storing question data
    
    This table stores individual problems/questions that are associated
    with specific files through the file_uuid foreign key
    """
    __tablename__ = "problems"

    uuid = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)
    file_uuid = Column(UUID(as_uuid=True), ForeignKey('file_metadata.uuid'), nullable=False, index=True)
    problem_index = Column(String(100), nullable=True, comment="Problem index within the file")
    problem_id = Column(String(100), nullable=True, comment="Problem identifier")
    problem_content = Column(String(1000), nullable=True, comment="Problem content")
    question_id = Column(Integer, nullable=True, comment="Question ID")
    question = Column(Text, nullable=False, comment="The question text")
    choices = Column(JSONType, nullable=False, comment="List of choice strings")
    answer = Column(Integer, nullable=False, comment="Index of correct answer (0, 1, 2, etc.)")
    explanation = Column(Text, nullable=True, comment="Explanation of the answer")
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationship to file_metadata
    file_metadata = relationship("FileMetadataModel", back_populates="problems")

    def __repr__(self):
        return f"<Problem(uuid={self.uuid}, question={self.question[:50]}...)>" 