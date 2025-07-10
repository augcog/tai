"""
File metadata service for business logic operations
"""

from typing import Optional, Dict
from sqlalchemy.orm import Session
import uuid

from app.core.models.file_metadata import FileMetadataModel
from app.schemas.file_metadata import FileMetadataCreate, FileMetadataUpdate, FileMetadataListParams


class FileMetadataService:
    """Service class for file metadata management operations"""

    @staticmethod
    def create_file_metadata(db: Session, metadata_data: FileMetadataCreate) -> FileMetadataModel:
        """Create new file metadata"""
        # Convert Pydantic models to dictionaries for JSON serialization
        sections_dict = None
        if metadata_data.sections:
            sections_dict = [section.dict() for section in metadata_data.sections]
        
        db_metadata = FileMetadataModel(
            file_name=metadata_data.file_name,
            url=metadata_data.url,
            sections=sections_dict,
        )
        db.add(db_metadata)
        db.commit()
        db.refresh(db_metadata)
        return db_metadata

    @staticmethod
    def get_file_metadata_by_uuid(db: Session, metadata_uuid: str) -> Optional[FileMetadataModel]:
        """Get file metadata by UUID"""
        try:
            uuid_obj = uuid.UUID(metadata_uuid)
            return db.query(FileMetadataModel).filter(FileMetadataModel.uuid == uuid_obj).first()
        except ValueError:
            return None

    @staticmethod
    def get_file_metadata_by_name(db: Session, file_name: str) -> Optional[FileMetadataModel]:
        """Get file metadata by file name"""
        return db.query(FileMetadataModel).filter(FileMetadataModel.file_name == file_name).first()

    @staticmethod
    def update_file_metadata(db: Session, metadata_uuid: str, metadata_data: FileMetadataUpdate) -> Optional[FileMetadataModel]:
        """Update file metadata information"""
        db_metadata = FileMetadataService.get_file_metadata_by_uuid(db, metadata_uuid)
        if not db_metadata:
            return None

        # Update only provided fields
        update_data = metadata_data.dict(exclude_unset=True)
        
        # Convert Pydantic models to dictionaries for JSON serialization
        if 'sections' in update_data and update_data['sections'] is not None:
            update_data['sections'] = [
                section.dict() if hasattr(section, "dict") else section
                for section in update_data['sections']
            ]
        
        for field, value in update_data.items():
            setattr(db_metadata, field, value)

        db.commit()
        db.refresh(db_metadata)
        return db_metadata

    @staticmethod
    def delete_file_metadata(db: Session, metadata_uuid: str) -> bool:
        """Delete file metadata"""
        db_metadata = FileMetadataService.get_file_metadata_by_uuid(db, metadata_uuid)
        if not db_metadata:
            return False

        db.delete(db_metadata)
        db.commit()
        return True

    @staticmethod
    def list_file_metadata(db: Session, params: FileMetadataListParams) -> Dict:
        """List file metadata with filtering and pagination"""
        query = db.query(FileMetadataModel)

        # Apply filters
        if params.search:
            search_term = f"%{params.search}%"
            query = query.filter(FileMetadataModel.file_name.ilike(search_term))

        # Get total count
        total_count = query.count()

        # Apply pagination
        offset = (params.page - 1) * params.limit
        files = query.offset(offset).limit(params.limit).all()

        # Calculate pagination info
        has_next = offset + params.limit < total_count
        has_prev = params.page > 1

        return {
            "files": files,
            "total_count": total_count,
            "page": params.page,
            "limit": params.limit,
            "has_next": has_next,
            "has_prev": has_prev,
        } 