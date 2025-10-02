"""
Course Cleanup Service for RAG System

This service handles the complete cleanup of course data from the database,
including all related chunks, files, and problems.
"""

import sqlite3
import logging
from pathlib import Path
from typing import Optional, Dict, List, Tuple
import json

logger = logging.getLogger(__name__)


class CourseCleanupService:
    """Service for cleaning up course data from the RAG database."""
    
    def __init__(self, database_path: str):
        """
        Initialize the cleanup service.
        
        Args:
            database_path: Path to the SQLite database file
        """
        self.database_path = database_path
        self._validate_database()
    
    def _validate_database(self):
        """Validate that the database exists and is accessible."""
        db_path = Path(self.database_path)
        if not db_path.exists():
            raise FileNotFoundError(f"Database not found: {self.database_path}")
        if not db_path.is_file():
            raise ValueError(f"Path is not a file: {self.database_path}")
        
    def _get_connection(self) -> sqlite3.Connection:
        """
        Get a database connection with row factory enabled.
        
        Returns:
            SQLite connection object
        """
        conn = sqlite3.connect(self.database_path)
        conn.row_factory = sqlite3.Row
        # Enable foreign key constraints for cascade deletes
        conn.execute("PRAGMA foreign_keys = ON")
        return conn
    
    def get_course_statistics(self, course_code: Optional[str] = None, 
                            course_name: Optional[str] = None) -> Dict:
        """
        Get statistics about course data in the database.
        
        Args:
            course_code: Course code to search for (e.g., "CS 61A")
            course_name: Course name to search for
            
        Returns:
            Dictionary with statistics about files, chunks, and problems
        """
        if not course_code and not course_name:
            raise ValueError("Either course_code or course_name must be provided")
        
        conn = self._get_connection()
        try:
            stats = {}
            
            # Build WHERE clause
            conditions = []
            params = []
            if course_code:
                conditions.append("course_code = ?")
                params.append(course_code)
            if course_name:
                conditions.append("course_name = ?")
                params.append(course_name)
            where_clause = " AND ".join(conditions)
            
            # Count files
            cursor = conn.execute(
                f"SELECT COUNT(*) as count FROM file WHERE {where_clause}",
                params
            )
            stats['file_count'] = cursor.fetchone()['count']
            
            # Get file UUIDs for chunk counting
            cursor = conn.execute(
                f"SELECT uuid FROM file WHERE {where_clause}",
                params
            )
            file_uuids = [row['uuid'] for row in cursor.fetchall()]
            
            # Count chunks
            if file_uuids:
                placeholders = ','.join(['?' for _ in file_uuids])
                cursor = conn.execute(
                    f"SELECT COUNT(*) as count FROM chunks WHERE file_uuid IN ({placeholders})",
                    file_uuids
                )
                stats['chunk_count'] = cursor.fetchone()['count']
                
                # Count all problems (including comprehensive questions)
                cursor = conn.execute(
                    f"SELECT COUNT(*) as count FROM problem WHERE file_uuid IN ({placeholders})",
                    file_uuids
                )
                stats['problem_count'] = cursor.fetchone()['count']
                
                # Count comprehensive questions specifically
                cursor = conn.execute(
                    f"SELECT COUNT(*) as count FROM problem WHERE file_uuid IN ({placeholders}) AND problem_id LIKE 'comprehensive_%'",
                    file_uuids
                )
                stats['comprehensive_question_count'] = cursor.fetchone()['count']
            else:
                stats['chunk_count'] = 0
                stats['problem_count'] = 0
                stats['comprehensive_question_count'] = 0
            
            # Get sample of file names
            cursor = conn.execute(
                f"SELECT file_name, relative_path FROM file WHERE {where_clause} LIMIT 5",
                params
            )
            stats['sample_files'] = [
                {'name': row['file_name'], 'path': row['relative_path']} 
                for row in cursor.fetchall()
            ]
            
            return stats
            
        finally:
            conn.close()
    
    def list_courses(self) -> List[Dict[str, str]]:
        """
        List all unique courses in the database.
        
        Returns:
            List of dictionaries with course_code and course_name
        """
        conn = self._get_connection()
        try:
            cursor = conn.execute("""
                SELECT DISTINCT course_code, course_name 
                FROM file 
                WHERE course_code IS NOT NULL OR course_name IS NOT NULL
                ORDER BY course_code, course_name
            """)
            
            courses = []
            for row in cursor.fetchall():
                courses.append({
                    'course_code': row['course_code'],
                    'course_name': row['course_name']
                })
            
            return courses
            
        finally:
            conn.close()
    
    def cleanup_course(self, course_code: Optional[str] = None,
                      course_name: Optional[str] = None,
                      dry_run: bool = False) -> Dict:
        """
        Remove all data related to a specific course from the database.
        
        This will delete:
        - All files associated with the course
        - All chunks from those files (via CASCADE)
        - All problems from those files (via CASCADE)
        
        Args:
            course_code: Course code to delete (e.g., "CS 61A")
            course_name: Course name to delete
            dry_run: If True, only simulate deletion and return what would be deleted
            
        Returns:
            Dictionary with deletion results/statistics
        """
        if not course_code and not course_name:
            raise ValueError("Either course_code or course_name must be provided")
        
        conn = self._get_connection()
        try:
            # Start transaction
            conn.execute("BEGIN TRANSACTION")
            
            # Build WHERE clause
            conditions = []
            params = []
            if course_code:
                conditions.append("course_code = ?")
                params.append(course_code)
            if course_name:
                conditions.append("course_name = ?")
                params.append(course_name)
            where_clause = " AND ".join(conditions)
            
            # Get statistics before deletion
            stats_before = self.get_course_statistics(course_code, course_name)
            
            # Get file UUIDs that will be deleted
            cursor = conn.execute(
                f"SELECT uuid, file_name, relative_path FROM file WHERE {where_clause}",
                params
            )
            files_to_delete = [
                {
                    'uuid': row['uuid'],
                    'name': row['file_name'],
                    'path': row['relative_path']
                }
                for row in cursor.fetchall()
            ]
            
            result = {
                'course_code': course_code,
                'course_name': course_name,
                'dry_run': dry_run,
                'statistics_before': stats_before,
                'files_deleted': files_to_delete
            }
            
            if not dry_run and files_to_delete:
                # Delete files (chunks and problems will cascade)
                cursor = conn.execute(
                    f"DELETE FROM file WHERE {where_clause}",
                    params
                )
                files_deleted_count = cursor.rowcount
                
                # Also clean up any orphaned chunks with matching course info
                # (in case of data inconsistency)
                cursor = conn.execute(
                    f"DELETE FROM chunks WHERE {where_clause}",
                    params
                )
                orphaned_chunks_deleted = cursor.rowcount
                
                # Commit transaction
                conn.commit()
                
                result['files_deleted_count'] = files_deleted_count
                result['orphaned_chunks_deleted'] = orphaned_chunks_deleted
                result['success'] = True
                result['message'] = f"Successfully deleted {files_deleted_count} files and related data"
                
                logger.info(f"Deleted course data: {course_code or course_name} - "
                          f"{files_deleted_count} files, {orphaned_chunks_deleted} orphaned chunks")
            else:
                # Rollback if dry run
                conn.rollback()
                
                if dry_run:
                    result['success'] = True
                    result['message'] = f"Dry run complete. Would delete {len(files_to_delete)} files"
                else:
                    result['success'] = False
                    result['message'] = "No files found for the specified course"
            
            return result
            
        except Exception as e:
            conn.rollback()
            logger.error(f"Error during course cleanup: {str(e)}")
            raise
        finally:
            conn.close()
    
    def cleanup_by_file_pattern(self, pattern: str, dry_run: bool = False) -> Dict:
        """
        Remove files and related data matching a specific pattern.
        
        Args:
            pattern: SQL LIKE pattern for file paths (e.g., '%/CS61A/%')
            dry_run: If True, only simulate deletion
            
        Returns:
            Dictionary with deletion results
        """
        conn = self._get_connection()
        try:
            conn.execute("BEGIN TRANSACTION")
            
            # Find matching files
            cursor = conn.execute(
                "SELECT uuid, file_name, relative_path, course_code, course_name "
                "FROM file WHERE relative_path LIKE ?",
                (pattern,)
            )
            
            files_to_delete = []
            for row in cursor.fetchall():
                files_to_delete.append({
                    'uuid': row['uuid'],
                    'name': row['file_name'],
                    'path': row['relative_path'],
                    'course_code': row['course_code'],
                    'course_name': row['course_name']
                })
            
            # Count related chunks and problems
            chunk_count = 0
            problem_count = 0
            if files_to_delete:
                file_uuids = [f['uuid'] for f in files_to_delete]
                placeholders = ','.join(['?' for _ in file_uuids])
                
                cursor = conn.execute(
                    f"SELECT COUNT(*) as count FROM chunks WHERE file_uuid IN ({placeholders})",
                    file_uuids
                )
                chunk_count = cursor.fetchone()['count']
                
                cursor = conn.execute(
                    f"SELECT COUNT(*) as count FROM problem WHERE file_uuid IN ({placeholders})",
                    file_uuids
                )
                problem_count = cursor.fetchone()['count']
            
            result = {
                'pattern': pattern,
                'dry_run': dry_run,
                'files_found': len(files_to_delete),
                'chunks_affected': chunk_count,
                'problems_affected': problem_count,
                'files': files_to_delete[:10]  # Limit to first 10 for display
            }
            
            if not dry_run and files_to_delete:
                # Delete files (cascades to chunks and problems)
                cursor = conn.execute(
                    "DELETE FROM file WHERE relative_path LIKE ?",
                    (pattern,)
                )
                files_deleted = cursor.rowcount
                
                conn.commit()
                
                result['files_deleted'] = files_deleted
                result['success'] = True
                result['message'] = f"Deleted {files_deleted} files and related data"
                
                logger.info(f"Deleted files matching pattern '{pattern}': {files_deleted} files")
            else:
                conn.rollback()
                
                if dry_run:
                    result['success'] = True
                    result['message'] = f"Dry run: Would delete {len(files_to_delete)} files"
                else:
                    result['success'] = False
                    result['message'] = "No files found matching the pattern"
            
            return result
            
        except Exception as e:
            conn.rollback()
            logger.error(f"Error during pattern cleanup: {str(e)}")
            raise
        finally:
            conn.close()
    
    def vacuum_database(self) -> Dict:
        """
        Vacuum the database to reclaim space after deletions.
        
        Returns:
            Dictionary with vacuum results
        """
        conn = self._get_connection()
        try:
            # Get size before vacuum
            cursor = conn.execute("SELECT page_count * page_size as size FROM pragma_page_count(), pragma_page_size()")
            size_before = cursor.fetchone()['size']
            
            # Vacuum
            conn.execute("VACUUM")
            
            # Get size after vacuum
            cursor = conn.execute("SELECT page_count * page_size as size FROM pragma_page_count(), pragma_page_size()")
            size_after = cursor.fetchone()['size']
            
            space_saved = size_before - size_after
            
            result = {
                'success': True,
                'size_before_bytes': size_before,
                'size_after_bytes': size_after,
                'space_saved_bytes': space_saved,
                'space_saved_mb': round(space_saved / (1024 * 1024), 2)
            }
            
            logger.info(f"Database vacuum complete. Space saved: {result['space_saved_mb']} MB")
            
            return result
            
        except Exception as e:
            logger.error(f"Error during vacuum: {str(e)}")
            raise
        finally:
            conn.close()
    
    def get_database_info(self) -> Dict:
        """
        Get general information about the database.
        
        Returns:
            Dictionary with database statistics
        """
        conn = self._get_connection()
        try:
            info = {}
            
            # Total counts
            cursor = conn.execute("SELECT COUNT(*) as count FROM file")
            info['total_files'] = cursor.fetchone()['count']
            
            cursor = conn.execute("SELECT COUNT(*) as count FROM chunks")
            info['total_chunks'] = cursor.fetchone()['count']
            
            cursor = conn.execute("SELECT COUNT(*) as count FROM problem")
            info['total_problems'] = cursor.fetchone()['count']
            
            # Database size
            cursor = conn.execute("SELECT page_count * page_size as size FROM pragma_page_count(), pragma_page_size()")
            size_bytes = cursor.fetchone()['size']
            info['database_size_bytes'] = size_bytes
            info['database_size_mb'] = round(size_bytes / (1024 * 1024), 2)
            
            # Course summary
            cursor = conn.execute("""
                SELECT course_code, course_name, COUNT(*) as file_count
                FROM file
                WHERE course_code IS NOT NULL OR course_name IS NOT NULL
                GROUP BY course_code, course_name
                ORDER BY file_count DESC
                LIMIT 10
            """)
            
            info['top_courses'] = []
            for row in cursor.fetchall():
                info['top_courses'].append({
                    'course_code': row['course_code'],
                    'course_name': row['course_name'],
                    'file_count': row['file_count']
                })
            
            return info
            
        finally:
            conn.close()

