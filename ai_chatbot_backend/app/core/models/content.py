"""
Models for content database (chunks, file and problem)
"""

from sqlalchemy import Column, String, Integer, Float, Text
from app.core.dbs.content_db import ContentBase

class ContentChunksModel(ContentBase):
    """Chunks model for metadata database"""
    __tablename__ = "chunks"

    chunk_uuid = Column(String, primary_key=True, index=True)
    file_uuid = Column(String, nullable=True) # FK → file(uuid)
    idx = Column(Integer, nullable=True)
    text = Column(Text, nullable=True)
    title = Column(String, nullable=True)
    url = Column(String, nullable=True)
    file_path = Column(String, nullable=True)
    reference_path = Column(String, nullable=True)  # Path to the original file
    course_name = Column(String, nullable=True)  # Course name for context
    course_id = Column(String, nullable=True)  # Course ID for context
    chunk_index = Column(Integer, nullable=True)  # Index of the chunk in the file
    vector = Column(Text, nullable=True)  # JSON blob for vector data

    def __repr__(self):
        return f"<Chunks(chunk_uuid={self.chunk_uuid}, file_uuid={self.file_uuid}, chunk_index={self.chunk_index})>"

class ContentFileModel(ContentBase):
    """File model for metadata database"""
    __tablename__ = "file"

    file_uuid = Column(String, primary_key=True, index=True)
    file_hash = Column(String, nullable=True)
    sections = Column(Text, nullable=True)  # JSON blob
    file_path = Column(String, default="")
    course_id = Column(String, default="")
    course_name = Column(String, default="")
    file_name = Column(String, nullable=True)
    extra_info = Column(Text, nullable=True)  # JSON blob for transcript, OCR, etc.

    def __repr__(self):
        return f"<File(file_uuid={self.file_uuid}, file_hash={self.file_hash}, course_id={self.course_id})>"


class ContentProblemModel(ContentBase):
    """Problem model for metadata database"""
    __tablename__ = "problem"

    uuid = Column(String, primary_key=True, index=True)
    file_uuid = Column(String, nullable=True)  # FK → file(uuid)
    problem_index = Column(Float, nullable=True)
    problem_id = Column(String, nullable=True)
    problem_content = Column(Text, nullable=True)
    question_id = Column(Integer, nullable=True)
    question = Column(Text, nullable=True)
    choices = Column(Text, nullable=True)  # JSON list[str]
    answer = Column(Text, nullable=True)   # JSON list[int]
    explanation = Column(Text, nullable=True)

    def __repr__(self):
        return f"<Problem(uuid={self.uuid}, file_uuid={self.file_uuid}, question_id={self.question_id})>"