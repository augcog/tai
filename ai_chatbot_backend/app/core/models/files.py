"""
Clean, simplified file models for modern API
Focused on essential functionality without over-engineering
"""

import uuid
from datetime import datetime
from typing import Optional

from sqlalchemy import Column, String, Integer, DateTime, Text, Boolean, Index
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func

from app.core.database import Base


class FileRegistry(Base):
    """
    Clean file registry for UUID-based file management

    Simplified design focused on essential functionality:
    - UUID-based secure access
    - Basic metadata (course, category, title)
    - File properties (size, type, timestamps)
    - No over-engineering (no assignment_number, week_number, subcategory)
    """

    __tablename__ = "file_registry"

    # Primary UUID identifier for API access
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)

    # File identification and location
    file_name = Column(String(255), nullable=False, index=True)
    relative_path = Column(String(500), nullable=False, unique=True, index=True)

    # File metadata
    mime_type = Column(String(100), nullable=False)
    size_bytes = Column(Integer, nullable=False)

    # Simple organization (no over-engineering)
    course_code = Column(String(20), nullable=True, index=True)
    # document, video, audio, other
    category = Column(String(50), nullable=True, index=True)

    # Content metadata
    title = Column(String(255), nullable=True)

    # Status
    is_active = Column(Boolean, default=True, nullable=False, index=True)

    # Timestamps
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    modified_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Performance indexes for common queries
    __table_args__ = (
        Index("idx_file_course_category", "course_code", "category"),
        Index("idx_file_active", "is_active"),
        Index("idx_file_created", "created_at"),
    )

    def __repr__(self):
        return f"<FileRegistry(id={self.id}, name='{self.file_name}', course='{self.course_code}')>"

    @property
    def display_name(self) -> str:
        """Get display name (title if available, otherwise filename)"""
        return self.title or self.file_name

    @property
    def file_extension(self) -> str:
        """Get file extension"""
        return self.file_name.split(".")[-1].lower() if "." in self.file_name else ""
