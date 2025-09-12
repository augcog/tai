#!/usr/bin/env python3
"""
Move all ROAR academy course records from one database to another.
"""

import sqlite3
import logging
from pathlib import Path
from typing import List, Tuple, Any

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def move_roar_academy_records(
    source_db_path: str = "/home/bot/bot/yk/YK_final/courses_out1/metadata.db",
    target_db_path: str = "/home/bot/bot/yk/YK_final/courses_out/metadata.db",
    course_code: str = "ROAR Academy",
    delete_from_source: bool = False
) -> Tuple[int, int, int]:
    """
    Move all records with specified course_code from source database to target database.
    
    Args:
        source_db_path: Path to source database
        target_db_path: Path to target database
        course_code: Course code to filter records (default: "ROAR academy")
        delete_from_source: Whether to delete records from source after moving
        
    Returns:
        Tuple of (files_moved, chunks_moved, problems_moved)
    """
    
    # Verify database files exist
    if not Path(source_db_path).exists():
        raise FileNotFoundError(f"Source database not found: {source_db_path}")
    if not Path(target_db_path).exists():
        raise FileNotFoundError(f"Target database not found: {target_db_path}")
    
    source_conn = sqlite3.connect(source_db_path)
    target_conn = sqlite3.connect(target_db_path)
    
    try:
        source_cur = source_conn.cursor()
        target_cur = target_conn.cursor()
        
        # Start transaction on target database
        target_conn.execute("BEGIN TRANSACTION")
        
        # 1. Get all file UUIDs with the specified course_code
        logger.info(f"Finding files with course_code = '{course_code}'")
        source_cur.execute(
            "SELECT * FROM file WHERE course_code = ?",
            (course_code,)
        )
        files = source_cur.fetchall()
        file_uuids = [f[0] for f in files]  # First column is uuid
        
        logger.info(f"Found {len(files)} files with course_code = '{course_code}'")
        
        # 2. Move file records
        files_moved = 0
        for file_record in files:
            # Check if file already exists in target
            target_cur.execute("SELECT uuid FROM file WHERE uuid = ?", (file_record[0],))
            if target_cur.fetchone():
                logger.warning(f"File {file_record[0]} already exists in target, skipping")
                continue
                
            # The source has 10 columns (including vector BLOB), target has 9 columns
            # We need to skip the vector column (9th column, index 8)
            # Columns: uuid, file_hash, sections, relative_path, course_code, course_name, file_name, extra_info, vector, url
            # Target needs: uuid, file_hash, sections, relative_path, course_code, course_name, file_name, extra_info, url
            file_record_adjusted = file_record[:8] + (file_record[9],) if len(file_record) > 9 else file_record[:8] + (None,)
            
            # Insert into target database
            placeholders = ','.join(['?' for _ in file_record_adjusted])
            target_cur.execute(
                f"INSERT INTO file VALUES ({placeholders})",
                file_record_adjusted
            )
            files_moved += 1
        
        logger.info(f"Moved {files_moved} file records")
        
        # 3. Move chunk records
        chunks_moved = 0
        for file_uuid in file_uuids:
            source_cur.execute(
                "SELECT * FROM chunks WHERE file_uuid = ?",
                (file_uuid,)
            )
            chunks = source_cur.fetchall()
            
            for chunk_record in chunks:
                # Check if chunk already exists in target
                target_cur.execute("SELECT chunk_uuid FROM chunks WHERE chunk_uuid = ?", (chunk_record[0],))
                if target_cur.fetchone():
                    logger.warning(f"Chunk {chunk_record[0]} already exists in target, skipping")
                    continue
                    
                # The source chunks table may have 12 columns (with vector BLOB), target has 11
                # We need to skip the vector column if it exists (last column)
                if len(chunk_record) == 12:
                    # Remove the vector column (last column)
                    chunk_record_adjusted = chunk_record[:11]
                else:
                    chunk_record_adjusted = chunk_record
                    
                # Insert into target database
                placeholders = ','.join(['?' for _ in chunk_record_adjusted])
                target_cur.execute(
                    f"INSERT INTO chunks VALUES ({placeholders})",
                    chunk_record_adjusted
                )
                chunks_moved += 1
        
        logger.info(f"Moved {chunks_moved} chunk records")
        
        # 4. Move problem records (if they exist)
        problems_moved = 0
        try:
            for file_uuid in file_uuids:
                source_cur.execute(
                    "SELECT * FROM problem WHERE file_uuid = ?",
                    (file_uuid,)
                )
                problems = source_cur.fetchall()
                
                for problem_record in problems:
                    # Check if problem already exists in target
                    target_cur.execute("SELECT uuid FROM problem WHERE uuid = ?", (problem_record[0],))
                    if target_cur.fetchone():
                        logger.warning(f"Problem {problem_record[0]} already exists in target, skipping")
                        continue
                        
                    # Insert into target database
                    placeholders = ','.join(['?' for _ in problem_record])
                    target_cur.execute(
                        f"INSERT INTO problem VALUES ({placeholders})",
                        problem_record
                    )
                    problems_moved += 1
            
            logger.info(f"Moved {problems_moved} problem records")
        except sqlite3.OperationalError as e:
            logger.warning(f"Problem table might not exist or have issues: {e}")
        
        # Commit the transaction
        target_conn.commit()
        logger.info("Successfully committed all changes to target database")
        
        # 5. Delete from source if requested
        if delete_from_source and (files_moved > 0 or chunks_moved > 0):
            logger.info("Deleting moved records from source database...")
            source_conn.execute("BEGIN TRANSACTION")
            
            # Delete chunks first (foreign key constraint)
            for file_uuid in file_uuids:
                source_cur.execute("DELETE FROM chunks WHERE file_uuid = ?", (file_uuid,))
                
            # Delete problems
            try:
                for file_uuid in file_uuids:
                    source_cur.execute("DELETE FROM problem WHERE file_uuid = ?", (file_uuid,))
            except sqlite3.OperationalError:
                pass
                
            # Delete files
            source_cur.execute("DELETE FROM file WHERE course_code = ?", (course_code,))
            
            source_conn.commit()
            logger.info("Deleted moved records from source database")
        
        return files_moved, chunks_moved, problems_moved
        
    except Exception as e:
        logger.error(f"Error during migration: {e}")
        # Rollback both connections
        source_conn.rollback()
        target_conn.rollback()
        raise
        
    finally:
        source_conn.close()
        target_conn.close()


def verify_migration(
    source_db_path: str = "/home/bot/bot/yk/YK_final/courses_out1/metadata.db",
    target_db_path: str = "/home/bot/bot/yk/YK_final/courses_out/metadata.db",
    course_code: str = "ROAR Academy"
):
    """
    Verify the migration by checking record counts in both databases.
    """
    source_conn = sqlite3.connect(source_db_path)
    target_conn = sqlite3.connect(target_db_path)
    
    try:
        # Check source database
        source_cur = source_conn.cursor()
        source_cur.execute("SELECT COUNT(*) FROM file WHERE course_code = ?", (course_code,))
        source_files = source_cur.fetchone()[0]
        
        # Check target database
        target_cur = target_conn.cursor()
        target_cur.execute("SELECT COUNT(*) FROM file WHERE course_code = ?", (course_code,))
        target_files = target_cur.fetchone()[0]
        
        logger.info(f"Source database has {source_files} files with course_code = '{course_code}'")
        logger.info(f"Target database has {target_files} files with course_code = '{course_code}'")
        
        # Count chunks
        source_cur.execute("""
            SELECT COUNT(*) FROM chunks 
            WHERE file_uuid IN (SELECT uuid FROM file WHERE course_code = ?)
        """, (course_code,))
        source_chunks = source_cur.fetchone()[0]
        
        target_cur.execute("""
            SELECT COUNT(*) FROM chunks 
            WHERE file_uuid IN (SELECT uuid FROM file WHERE course_code = ?)
        """, (course_code,))
        target_chunks = target_cur.fetchone()[0]
        
        logger.info(f"Source database has {source_chunks} chunks for course_code = '{course_code}'")
        logger.info(f"Target database has {target_chunks} chunks for course_code = '{course_code}'")
        
        return {
            "source": {"files": source_files, "chunks": source_chunks},
            "target": {"files": target_files, "chunks": target_chunks}
        }
        
    finally:
        source_conn.close()
        target_conn.close()


if __name__ == "__main__":
    # Verify before migration
    logger.info("=== Before Migration ===")
    stats_before = verify_migration()
    
    # Perform migration
    logger.info("\n=== Starting Migration ===")
    files_moved, chunks_moved, problems_moved = move_roar_academy_records(
        delete_from_source=True  # Set to False if you want to keep records in source
    )
    
    logger.info(f"\n=== Migration Complete ===")
    logger.info(f"Files moved: {files_moved}")
    logger.info(f"Chunks moved: {chunks_moved}")
    logger.info(f"Problems moved: {problems_moved}")
    
    # Verify after migration
    logger.info("\n=== After Migration ===")
    stats_after = verify_migration()