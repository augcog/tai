#!/usr/bin/env python3
"""
Database Initialization Module

This module handles:
1. Database table creation
2. MongoDB data loading for fresh installations
"""

from pathlib import Path
from typing import Dict
import logging

from sqlalchemy import create_engine, inspect
from sqlalchemy.orm import sessionmaker

from app.core.dbs.course_db import Base, COURSE_DATABASE_URL
from app.core.dbs.metadata_db import (
    MetadataBase,
    metadata_engine,
    MetadataSessionLocal,
)
from app.core.models.courses import CourseModel
from app.core.models.metadata import FileModel, ProblemModel
from app.core.mongodb_client import get_mongodb_client, load_database_mapping
from app.config import settings

# Configure logging
logger = logging.getLogger(__name__)


class DatabaseInitializer:
    """Handles database initialization and MongoDB data loading"""

    def __init__(self):
        self.engine = create_engine(
            COURSE_DATABASE_URL, connect_args={"check_same_thread": False}
        )
        self.SessionLocal = sessionmaker(
            autocommit=False, autoflush=False, bind=self.engine
        )

    def initialize_database(self) -> bool:
        """
        Complete database initialization process
        Returns True if successful, False otherwise
        """
        try:
            logger.info("🚀 Starting database initialization...")

            # Step 1: Create tables if they don't exist
            if not self._create_tables():
                return False

            # Step 2: Check and run migrations if needed
            if not self._check_and_migrate():
                return False

            # Step 3: Check if local database is empty and load from MongoDB if needed
            if self._is_database_empty() and settings.MONGODB_ENABLED:
                if not self._load_from_mongodb():
                    logger.warning(
                        "⚠️ Failed to load data from MongoDB, continuing with local initialization"
                    )

            # Database initialization completed

            logger.info("✅ Database initialization completed successfully!")
            return True

        except Exception as e:
            logger.error(f"❌ Database initialization failed: {e}")
            return False

    def _create_tables(self) -> bool:
        """Create database tables if they don't exist"""
        try:
            logger.info("🏗️  Creating database tables...")

            # Create courses database tables (only CourseModel)
            Base.metadata.create_all(bind=self.engine)

            # Create metadata database tables (FileModel, ProblemModel)
            MetadataBase.metadata.create_all(bind=metadata_engine)

            # Verify tables were created
            inspector = inspect(self.engine)
            courses_tables = inspector.get_table_names()
            logger.info(f"📊 Courses database tables: {courses_tables}")

            metadata_inspector = inspect(metadata_engine)
            metadata_tables = metadata_inspector.get_table_names()
            logger.info(f"📊 Metadata database tables: {metadata_tables}")

            return True
        except Exception as e:
            logger.error(f"❌ Failed to create tables: {e}")
            return False

    def _check_and_migrate(self) -> bool:
        """Check if migration is needed and perform it"""
        try:
            # Check courses database - should have courses table
            inspector = inspect(self.engine)
            courses_tables = inspector.get_table_names()

            if "courses" in courses_tables:
                logger.info("✅ Courses table exists in courses database")

            # Check metadata database
            metadata_inspector = inspect(metadata_engine)
            metadata_tables = metadata_inspector.get_table_names()

            if "file" in metadata_tables and "problem" in metadata_tables:
                logger.info("✅ Metadata database already has correct schema")
            else:
                logger.info("🔄 Creating metadata database tables")
                MetadataBase.metadata.create_all(bind=metadata_engine)

            return True

        except Exception as e:
            logger.error(f"❌ Migration check failed: {e}")
            return False

    def _is_database_empty(self) -> bool:
        """Check if the local databases are empty (no courses or files)

        Returns:
            bool: True if databases are empty
        """
        try:
            # Check courses database
            session = self.SessionLocal()
            course_count = session.query(CourseModel).count()
            session.close()

            # Check metadata database
            metadata_session = MetadataSessionLocal()
            file_count = metadata_session.query(FileModel).count()
            metadata_session.close()

            is_empty = course_count == 0 and file_count == 0
            logger.info(
                f"🔍 Database status: {course_count} courses, {file_count} files (empty: {is_empty})"
            )

            return is_empty

        except Exception as e:
            logger.error(f"❌ Failed to check database status: {e}")
            return False

    def _load_from_mongodb(self) -> bool:
        """Load data from MongoDB into local databases

        Returns:
            bool: True if successful
        """
        try:
            logger.info("☁️ Loading data from MongoDB...")

            # Get MongoDB client
            mongodb_client = get_mongodb_client()

            # Test connection
            if not mongodb_client.connect():
                logger.error("❌ Failed to connect to MongoDB")
                return False

            # Load database mapping configuration
            config = load_database_mapping()
            if not config:
                logger.error("❌ Failed to load database mapping configuration")
                return False

            # Load courses from MongoDB
            courses_loaded = self._load_courses_from_mongodb(mongodb_client, config)

            # Load files from MongoDB
            files_loaded = self._load_files_from_mongodb(mongodb_client, config)

            # Load problems from MongoDB
            problems_loaded = self._load_problems_from_mongodb(mongodb_client, config)

            success = courses_loaded and files_loaded and problems_loaded

            if success:
                logger.info("✅ Successfully loaded data from MongoDB")
            else:
                logger.error("❌ Failed to load some data from MongoDB")

            return success

        except Exception as e:
            logger.error(f"❌ Failed to load data from MongoDB: {e}")
            return False

    def _load_courses_from_mongodb(self, mongodb_client, config: Dict) -> bool:
        """Load courses from MongoDB

        Args:
            mongodb_client: MongoDB client instance
            config: Database mapping configuration

        Returns:
            bool: True if successful
        """
        try:
            # Get courses from MongoDB
            courses_data = mongodb_client.find_all("courses", "courses")

            if not courses_data:
                logger.info("📚 No courses found in MongoDB")
                return True

            logger.info(f"📚 Loading {len(courses_data)} courses from MongoDB")

            session = self.SessionLocal()

            try:
                for course_data in courses_data:
                    # Convert MongoDB document to CourseModel
                    course = CourseModel(
                        course_name=course_data.get("course_name", "Unknown Course"),
                        course_code=course_data.get("course_code", "UNKNOWN"),
                        server_url=course_data.get("server_url", settings.SERVER_URL),
                        semester=course_data.get("semester", "Fall 2024"),
                        enabled=course_data.get("enabled", True),
                        order=course_data.get("order", 0),
                        access_type=course_data.get("access_type", "public"),
                        school=course_data.get("school"),
                    )

                    session.add(course)

                session.commit()
                logger.info(
                    f"✅ Successfully loaded {len(courses_data)} courses from MongoDB"
                )
                return True

            except Exception as e:
                session.rollback()
                logger.error(f"❌ Failed to save courses to local database: {e}")
                return False
            finally:
                session.close()

        except Exception as e:
            logger.error(f"❌ Failed to load courses from MongoDB: {e}")
            return False

    def _load_files_from_mongodb(self, mongodb_client, config: Dict) -> bool:
        """Load files from MongoDB into metadata database

        Args:
            mongodb_client: MongoDB client instance
            config: Database mapping configuration

        Returns:
            bool: True if successful
        """
        try:
            # Get files from MongoDB
            files_data = mongodb_client.find_all("metadata", "file")

            if not files_data:
                logger.info("📁 No files found in MongoDB")
                return True

            logger.info(f"📁 Loading {len(files_data)} files from MongoDB")

            session = MetadataSessionLocal()

            try:
                for file_data in files_data:
                    # Convert MongoDB document to FileModel
                    file_record = FileModel(
                        uuid=file_data.get("uuid"),
                        file_name=file_data.get("file_name", "unknown"),
                        url=file_data.get("url"),
                        sections=file_data.get("sections"),
                        relative_path=file_data.get("relative_path", ""),
                        course_code=file_data.get("course_code", ""),
                        course_name=file_data.get("course_name", ""),
                    )

                    session.add(file_record)

                session.commit()
                logger.info(
                    f"✅ Successfully loaded {len(files_data)} files from MongoDB"
                )
                return True

            except Exception as e:
                session.rollback()
                logger.error(f"❌ Failed to save files to metadata database: {e}")
                return False
            finally:
                session.close()

        except Exception as e:
            logger.error(f"❌ Failed to load files from MongoDB: {e}")
            return False

    def _load_problems_from_mongodb(self, mongodb_client, config: Dict) -> bool:
        """Load problems from MongoDB into metadata database

        Args:
            mongodb_client: MongoDB client instance
            config: Database mapping configuration

        Returns:
            bool: True if successful
        """
        try:
            # Get problems from MongoDB
            problems_data = mongodb_client.find_all("metadata", "problem")

            if not problems_data:
                logger.info("🧩 No problems found in MongoDB")
                return True

            logger.info(f"🧩 Loading {len(problems_data)} problems from MongoDB")

            session = MetadataSessionLocal()

            try:
                for problem_data in problems_data:
                    # Convert MongoDB document to ProblemModel
                    problem_record = ProblemModel(
                        uuid=problem_data.get("uuid"),
                        file_uuid=problem_data.get("file_uuid"),
                        problem_index=problem_data.get("problem_index"),
                        problem_id=problem_data.get("problem_id"),
                        problem_content=problem_data.get("problem_content"),
                        question_id=problem_data.get("question_id"),
                        question=problem_data.get("question"),
                        choices=problem_data.get("choices"),
                        answer=problem_data.get("answer"),
                        explanation=problem_data.get("explanation"),
                    )

                    session.add(problem_record)

                session.commit()
                logger.info(
                    f"✅ Successfully loaded {len(problems_data)} problems from MongoDB"
                )
                return True

            except Exception as e:
                session.rollback()
                logger.error(f"❌ Failed to save problems to metadata database: {e}")
                return False
            finally:
                session.close()

        except Exception as e:
            logger.error(f"❌ Failed to load problems from MongoDB: {e}")
            return False

    def get_database_status(self) -> Dict:
        """Get current database status"""
        try:
            session = self.SessionLocal()

            course_count = session.query(CourseModel).count()

            session.close()

            # Check database file size
            db_path = COURSE_DATABASE_URL.replace("sqlite:///", "")
            db_size = Path(db_path).stat().st_size if Path(db_path).exists() else 0

            return {
                "database_exists": Path(db_path).exists(),
                "database_size_bytes": db_size,
                "course_count": course_count,
            }

        except Exception as e:
            logger.error(f"❌ Failed to get database status: {e}")
            return {"error": str(e)}


# Global initializer instance
_initializer = None


def get_initializer() -> DatabaseInitializer:
    """Get global database initializer instance"""
    global _initializer
    if _initializer is None:
        _initializer = DatabaseInitializer()
    return _initializer


def initialize_database_on_startup() -> bool:
    """
    Initialize database on server startup
    This function should be called from main.py
    """
    initializer = get_initializer()
    return initializer.initialize_database()