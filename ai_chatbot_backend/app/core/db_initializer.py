#!/usr/bin/env python3
"""
Database Initialization and File Import Module

This module handles:
1. Database table creation
2. Migration from legacy schemas
3. Automatic import of existing files from data/ directory
4. Course registration from directory structure
"""

import sqlite3
import mimetypes
import uuid
import json
from pathlib import Path
from datetime import datetime
from typing import List, Dict
import logging

from sqlalchemy import create_engine, inspect
from sqlalchemy.orm import sessionmaker

from app.core.database import Base, SQLALCHEMY_DATABASE_URL
from app.core.models.courses import CourseModel
from app.core.models.files import FileRegistry

# Configure logging
logger = logging.getLogger(__name__)


class DatabaseInitializer:
    """Handles database initialization and file importing"""

    def __init__(self, data_dir: str = "data"):
        self.data_dir = Path(data_dir)
        self.engine = create_engine(
            SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
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

            # Step 3: Import existing files from data directory
            if self.data_dir.exists():
                imported_count = self._import_existing_files()
                logger.info(f"📁 Imported {imported_count} files from data directory")
            else:
                logger.info(
                    f"📁 Data directory {self.data_dir} does not exist, skipping file import"
                )

            # Step 4: Update course registry
            course_count = self._update_course_registry()
            logger.info(f"📚 Registered {course_count} courses")

            logger.info("✅ Database initialization completed successfully!")
            return True

        except Exception as e:
            logger.error(f"❌ Database initialization failed: {e}")
            return False

    def _create_tables(self) -> bool:
        """Create database tables if they don't exist"""
        try:
            logger.info("🏗️  Creating database tables...")
            Base.metadata.create_all(bind=self.engine)

            # Verify tables were created
            inspector = inspect(self.engine)
            tables = inspector.get_table_names()
            logger.info(f"📊 Available tables: {tables}")

            return True
        except Exception as e:
            logger.error(f"❌ Failed to create tables: {e}")
            return False

    def _check_and_migrate(self) -> bool:
        """Check if migration is needed and perform it"""
        try:
            # Check if we have the modern schema
            inspector = inspect(self.engine)
            if "file_registry" not in inspector.get_table_names():
                logger.info("✅ No existing file_registry table, using modern schema")
                return True

            # Check column structure
            columns = [col["name"] for col in inspector.get_columns("file_registry")]
            has_modern_schema = "modified_at" in columns and "updated_at" not in columns

            if has_modern_schema:
                logger.info("✅ Database already has modern schema")
                return True
            else:
                logger.info("🔄 Legacy schema detected, running migration...")
                return self._run_migration()

        except Exception as e:
            logger.error(f"❌ Migration check failed: {e}")
            return False

    def _run_migration(self) -> bool:
        """Run database migration from legacy to modern schema"""
        try:
            # This is a simplified version of the migration
            # For complex migrations, you might want to use the full migrate_to_modern_files.py script

            db_path = SQLALCHEMY_DATABASE_URL.replace("sqlite:///", "")
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()

            # Backup existing data
            cursor.execute(
                """
                SELECT id, file_name, relative_path, mime_type, size_bytes,
                       course_code, category, title, is_active, created_at
                FROM file_registry WHERE is_active = 1
            """
            )
            existing_data = cursor.fetchall()

            if existing_data:
                logger.info(f"💾 Backing up {len(existing_data)} existing files...")
                cursor.execute(
                    "ALTER TABLE file_registry RENAME TO file_registry_backup"
                )
                conn.commit()

            conn.close()

            # Create new schema
            Base.metadata.create_all(bind=self.engine)

            # Migrate data if we had any
            if existing_data:
                session = self.SessionLocal()
                try:
                    for row in existing_data:
                        file_record = FileRegistry(
                            id=row[0] if row[0] else uuid.uuid4(),
                            file_name=row[1],
                            relative_path=row[2],
                            mime_type=row[3],
                            size_bytes=row[4],
                            course_code=row[5],
                            category=(
                                row[6]
                                if row[6] in ["document", "video", "audio", "other"]
                                else "other"
                            ),
                            title=row[7],
                            is_active=bool(row[8]) if row[8] is not None else True,
                            created_at=(
                                datetime.fromisoformat(row[9])
                                if row[9]
                                else datetime.now()
                            ),
                        )
                        session.add(file_record)

                    session.commit()
                    logger.info(
                        f"✅ Migrated {len(existing_data)} files to modern schema"
                    )
                except Exception as e:
                    session.rollback()
                    logger.error(f"❌ Migration failed: {e}")
                    return False
                finally:
                    session.close()

            return True

        except Exception as e:
            logger.error(f"❌ Migration failed: {e}")
            return False

    def _import_existing_files(self) -> int:
        """Import existing files from data directory into database"""
        session = self.SessionLocal()
        imported_count = 0

        try:
            # Get list of files already in database to avoid duplicates
            existing_files = set()
            for file_record in session.query(FileRegistry).all():
                existing_files.add(file_record.relative_path)

            # Scan data directory for files
            for file_path in self._scan_data_directory():
                # Convert to absolute path first, then get relative path from the project root
                abs_file_path = file_path.resolve()
                project_root = Path.cwd().resolve()

                try:
                    # Calculate relative path from project root
                    relative_path = str(abs_file_path.relative_to(project_root))
                except ValueError:
                    # If file is not under project root, use the path relative to data directory
                    try:
                        relative_path = str(
                            abs_file_path.relative_to(self.data_dir.resolve())
                        )
                        relative_path = f"{self.data_dir.name}/{relative_path}"
                    except ValueError:
                        # Fall back to absolute path if all else fails
                        relative_path = str(abs_file_path)
                        logger.warning(f"Using absolute path for file: {relative_path}")

                # Skip if already in database
                if relative_path in existing_files:
                    continue

                # Extract metadata from file path
                metadata = self._extract_file_metadata(file_path)

                # Create file registry entry
                file_record = FileRegistry(
                    file_name=file_path.name,
                    relative_path=relative_path,
                    mime_type=metadata["mime_type"],
                    size_bytes=metadata["size_bytes"],
                    course_code=metadata["course_code"],
                    category=metadata["category"],
                    title=metadata["title"],
                    is_active=True,
                )

                session.add(file_record)
                imported_count += 1

                # Commit in batches
                if imported_count % 50 == 0:
                    session.commit()
                    logger.info(f"📁 Imported {imported_count} files so far...")

            session.commit()
            logger.info(f"✅ Successfully imported {imported_count} new files")

        except Exception as e:
            session.rollback()
            logger.error(f"❌ File import failed: {e}")
        finally:
            session.close()

        return imported_count

    def _scan_data_directory(self) -> List[Path]:
        """Scan data directory and return list of all files"""
        files = []

        if not self.data_dir.exists():
            return files

        # Recursively find all files
        for file_path in self.data_dir.rglob("*"):
            if file_path.is_file() and not file_path.name.startswith("."):
                files.append(file_path)

        return files

    def _extract_file_metadata(self, file_path: Path) -> Dict:
        """Extract metadata from file path and content"""
        # Get file stats
        stats = file_path.stat()

        # Determine MIME type
        mime_type, _ = mimetypes.guess_type(str(file_path))
        if not mime_type:
            mime_type = "application/octet-stream"

        # Extract course code from path (assumes data/COURSE_CODE/... structure)
        path_parts = file_path.parts
        course_code = None
        if len(path_parts) > 1 and path_parts[0] == "data":
            course_code = path_parts[1] if len(path_parts) > 1 else None

        # Determine category from path
        category = "other"
        if "documents" in path_parts:
            category = "document"
        elif "videos" in path_parts:
            category = "video"
        elif "audios" in path_parts:
            category = "audio"

        # Generate title from filename
        title = file_path.stem.replace("_", " ").replace("-", " ").title()

        return {
            "mime_type": mime_type,
            "size_bytes": stats.st_size,
            "course_code": course_code,
            "category": category,
            "title": title,
        }

    def _update_course_registry(self) -> int:
        """Load courses from JSON only if database is empty (fresh initialization)"""
        session = self.SessionLocal()

        try:
            # Check if there are already courses in the database
            existing_course_count = session.query(CourseModel).count()

            if existing_course_count > 0:
                logger.info(
                    f"📚 Found {existing_course_count} existing courses in database, skipping course loading"
                )
                return 0

            # Database is empty, proceed with JSON loading
            logger.info("📄 Database is empty, loading courses from course.json...")
            return self._load_courses_from_json(session)

        except Exception as e:
            logger.error(f"❌ Course registry check failed: {e}")
            return 0
        finally:
            session.close()

    def _load_courses_from_json(self, session) -> int:
        """Load courses from course.json file (only called when database is empty)"""
        course_count = 0

        try:
            course_json_path = Path("course.json")

            if not course_json_path.exists():
                logger.warning("📄 course.json not found, no courses will be loaded")
                return 0

            # Read JSON file
            with open(course_json_path, "r", encoding="utf-8") as f:
                courses_data = json.load(f)

            if not isinstance(courses_data, list):
                logger.error("❌ course.json must contain a list of courses")
                return 0

            # Load all courses from JSON (database is empty)
            for course_data in courses_data:
                try:
                    course = CourseModel(
                        course_name=course_data.get("course_name", "Unknown Course"),
                        course_code=course_data.get("course_code", "UNKNOWN"),
                        server_url=course_data.get(
                            "server_url", "http://128.32.43.233:8000"
                        ),
                        semester=course_data.get("semester", "Fall 2024"),
                        enabled=course_data.get("enabled", True),
                        order=course_data.get("order", course_count),
                        access_type=course_data.get("access_type", "public"),
                        school=course_data.get("school"),
                    )
                    session.add(course)
                    course_count += 1
                    logger.info(
                        f"📚 Loading course: {course.course_name} ({course.course_code})"
                    )

                except Exception as e:
                    logger.error(f"❌ Failed to load course {course_data}: {e}")
                    continue

            session.commit()
            logger.info(
                f"✅ Successfully loaded {course_count} courses from course.json"
            )

        except Exception as e:
            session.rollback()
            logger.error(f"❌ Failed to load courses from JSON: {e}")

        return course_count

    def get_database_status(self) -> Dict:
        """Get current database status"""
        try:
            session = self.SessionLocal()

            file_count = (
                session.query(FileRegistry)
                .filter(FileRegistry.is_active == True)
                .count()
            )
            course_count = session.query(CourseModel).count()

            session.close()

            # Check database file size
            db_path = SQLALCHEMY_DATABASE_URL.replace("sqlite:///", "")
            db_size = Path(db_path).stat().st_size if Path(db_path).exists() else 0

            return {
                "database_exists": Path(db_path).exists(),
                "database_size_bytes": db_size,
                "file_count": file_count,
                "course_count": course_count,
                "data_directory_exists": self.data_dir.exists(),
            }

        except Exception as e:
            logger.error(f"❌ Failed to get database status: {e}")
            return {"error": str(e)}


# Global initializer instance
_initializer = None


def get_initializer(data_dir: str = "data") -> DatabaseInitializer:
    """Get global database initializer instance"""
    global _initializer
    if _initializer is None:
        _initializer = DatabaseInitializer(data_dir)
    return _initializer


def initialize_database_on_startup(data_dir: str = "data") -> bool:
    """
    Initialize database on server startup
    This function should be called from main.py
    """
    initializer = get_initializer(data_dir)
    return initializer.initialize_database()
