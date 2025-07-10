"""
File metadata model for file information management
"""

from sqlalchemy import Column, String, Text, UniqueConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
import uuid
import sqlalchemy
from app.core.database import Base

# SQLite fallback for JSON
if hasattr(sqlalchemy, 'JSON'):
    JSONType = sqlalchemy.JSON
else:
    JSONType = Text

class FileMetadataModel(Base):
    """
    File metadata model for storing file information
    
    Features:
    - UUID-based secure access
    - File metadata (name, url, sections)
    - Simplified structure with only essential fields
    """

    __tablename__ = "file_metadata"
    __table_args__ = (
        UniqueConstraint("file_name", name="uq_file_metadata_file_name"),
    )

    uuid = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)
    file_name = Column(String(255), nullable=False, unique=True, index=True)
    url = Column(String(500), nullable=True)
    sections = Column(JSONType, nullable=True)

    # Relationship to problems
    problems = relationship("ProblemModel", back_populates="file_metadata")

    def __repr__(self):
        return f"<FileMetadata(uuid={self.uuid}, file_name={self.file_name})>" 