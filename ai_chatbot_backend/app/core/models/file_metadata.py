"""
File metadata model for file information management
"""

from sqlalchemy import Column, String, Text, UniqueConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
import uuid
import sqlalchemy
from app.core.database import PracticeBase

# SQLite fallback for JSON
if hasattr(sqlalchemy, 'JSON'):
    JSONType = sqlalchemy.JSON
else:
    JSONType = Text


class FileMetadataModel(PracticeBase):
    """
    File metadata model for storing file information

    Features:
    - UUID-based secure access
    - File metadata (name, url, sections)
    - Simplified structure with only essential fields
    """

    __tablename__ = "file"
    __table_args__ = (
        UniqueConstraint("file_name", name="uq_file_metadata_file_name"),
    )

    uuid = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()), index=True)
    file_name = Column(String(255), nullable=False, unique=True, index=True)
    url = Column(String(500), nullable=True)
    sections = Column(JSONType, nullable=True)
    relative_path = Column(String(255), nullable=True)
    course_code = Column(String(255), nullable=True)
    course_name = Column(String(255), nullable=True)

    # Relationship to problems
    problems = relationship("ProblemModel", back_populates="file_metadata")

    def __repr__(self):
        return f"<FileMetadata(uuid={self.uuid}, file_name={self.file_name})>"