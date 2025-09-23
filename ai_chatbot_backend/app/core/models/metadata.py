"""
Models for metadata database (files and problems)
"""

from sqlalchemy import Column, String, Integer, Float, Text
from app.core.dbs.metadata_db import MetadataBase


class FileModel(MetadataBase):
    """File model for metadata database"""
    __tablename__ = "file"

    uuid = Column(String, primary_key=True, index=True)
    file_name = Column(String, nullable=False)
    url = Column(String, nullable=True)
    sections = Column(Text, nullable=True)  # JSON blob
    extra_info = Column(Text, nullable=True)  # JSON blob for transcript, OCR, etc.
    relative_path = Column(String, default="")
    course_code = Column(String, default="")
    course_name = Column(String, default="")

    def __repr__(self):
        return f"<File(uuid={self.uuid}, name={self.file_name}, course={self.course_code})>"


class ProblemModel(MetadataBase):
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
    question_type = Column(String, nullable=True)

    def __repr__(self):
        return f"<Problem(uuid={self.uuid}, file_uuid={self.file_uuid}, question_id={self.question_id})>"