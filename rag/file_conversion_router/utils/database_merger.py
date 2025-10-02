"""Database merging utilities for combining course databases into collective databases."""

import logging
import sqlite3
from pathlib import Path
from typing import List, Dict

from file_conversion_router.services.directory_service import SQL_INIT


def merge_course_databases_into_collective(
    course_db_paths: List[str],
    collective_db_path: str,
) -> Dict[str, int]:
    """Merge multiple small course databases into a collective database.

    Args:
        course_db_paths: List of paths to individual course database files
        collective_db_path: Path to the collective/master database

    Returns:
        Dictionary with merge statistics (files, chunks, problems merged per course)
    """
    collective_db_path = Path(collective_db_path)

    # Ensure collective database exists and has proper schema
    collective_db_path.parent.mkdir(parents=True, exist_ok=True)
    collective_conn = sqlite3.connect(str(collective_db_path))
    collective_conn.row_factory = sqlite3.Row

    # Initialize schema in collective database
    collective_conn.executescript(SQL_INIT)

    merge_stats = {}

    try:
        for course_db_path in course_db_paths:
            course_db_path = Path(course_db_path)

            if not course_db_path.exists():
                logging.warning(f"Course database not found: {course_db_path}")
                continue

            logging.info(f"Merging course database: {course_db_path}")

            # Connect to course database
            course_conn = sqlite3.connect(str(course_db_path))
            course_conn.row_factory = sqlite3.Row

            try:
                # Get course statistics
                stats = _merge_single_course_db(course_conn, collective_conn, str(course_db_path))
                merge_stats[str(course_db_path)] = stats

                logging.info(f"Merged {course_db_path.name}: {stats}")

            except Exception as e:
                logging.error(f"Failed to merge {course_db_path}: {str(e)}")
                continue
            finally:
                course_conn.close()

        collective_conn.commit()
        logging.info(f"Successfully merged {len(merge_stats)} databases into collective database")

    except Exception as e:
        collective_conn.rollback()
        logging.error(f"Error during merge process: {str(e)}")
        raise
    finally:
        collective_conn.close()

    return merge_stats


def _delete_course_data_from_collective(
    collective_conn: sqlite3.Connection,
    course_code: str
) -> Dict[str, int]:
    """Delete all data for a specific course from the collective database.

    Args:
        collective_conn: Connection to the collective database
        course_code: The course code to delete data for

    Returns:
        Dictionary with deletion statistics
    """
    deletion_stats = {"files_deleted": 0, "chunks_deleted": 0, "problems_deleted": 0}

    try:
        # Get count of files to be deleted
        files_count = collective_conn.execute(
            "SELECT COUNT(*) FROM file WHERE course_code = ?", (course_code,)
        ).fetchone()[0]
        deletion_stats["files_deleted"] = files_count

        # Get UUIDs of files to be deleted for cascading deletes
        file_uuids = collective_conn.execute(
            "SELECT uuid FROM file WHERE course_code = ?", (course_code,)
        ).fetchall()

        file_uuid_list = [row[0] for row in file_uuids]

        if file_uuid_list:
            # Count chunks and problems to be deleted
            placeholders = ",".join("?" * len(file_uuid_list))

            chunks_count = collective_conn.execute(
                f"SELECT COUNT(*) FROM chunks WHERE file_uuid IN ({placeholders})",
                file_uuid_list
            ).fetchone()[0]
            deletion_stats["chunks_deleted"] = chunks_count

            problems_count = collective_conn.execute(
                f"SELECT COUNT(*) FROM problem WHERE file_uuid IN ({placeholders})",
                file_uuid_list
            ).fetchone()[0]
            deletion_stats["problems_deleted"] = problems_count

            # Delete chunks first (referential integrity)
            collective_conn.execute(
                f"DELETE FROM chunks WHERE file_uuid IN ({placeholders})",
                file_uuid_list
            )

            # Delete problems
            collective_conn.execute(
                f"DELETE FROM problem WHERE file_uuid IN ({placeholders})",
                file_uuid_list
            )

        # Finally delete files
        collective_conn.execute(
            "DELETE FROM file WHERE course_code = ?", (course_code,)
        )

        logging.info(f"Deleted existing course data for {course_code}: {deletion_stats}")

    except Exception as e:
        logging.error(f"Failed to delete course data for {course_code}: {str(e)}")
        raise

    return deletion_stats


def _merge_single_course_db(
    course_conn: sqlite3.Connection,
    collective_conn: sqlite3.Connection,
    course_db_name: str
) -> Dict[str, int]:
    """Merge a single course database into the collective database.

    First deletes all existing data for the course, then adds all content from course db.

    Returns statistics about the merge operation.
    """
    # Ensure both connections have row factory set
    course_conn.row_factory = sqlite3.Row
    collective_conn.row_factory = sqlite3.Row

    stats = {"files": 0, "chunks": 0, "problems": 0, "conflicts": 0, "deleted": {}}

    # Get all files from course database to determine course_code
    files = course_conn.execute("SELECT * FROM file").fetchall()

    if not files:
        logging.warning(f"No files found in course database: {course_db_name}")
        return stats

    # Get course_code from the first file to delete existing data
    course_code = files[0]['course_code']

    # Delete all existing data for this course from collective database
    deletion_stats = _delete_course_data_from_collective(collective_conn, course_code)
    stats["deleted"] = deletion_stats

    # Since we deleted all existing data for this course, simply insert all files
    for file_row in files:
        file_uuid = file_row['uuid']

        # Insert file (no need to check for conflicts since we deleted all course data)
        # Preserve created_at and update_time if they exist, otherwise use current timestamp
        created_at = file_row['created_at'] if 'created_at' in file_row.keys() and file_row['created_at'] else None
        update_time = file_row['update_time'] if 'update_time' in file_row.keys() and file_row['update_time'] else None
        collective_conn.execute("""
            INSERT OR REPLACE INTO file (uuid, file_hash, sections, relative_path,
                                       course_code, course_name, file_name, extra_info, url,
                                       created_at, update_time)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, COALESCE(?, datetime('now', 'localtime')), COALESCE(?, datetime('now', 'localtime')))
        """, (
            file_row['uuid'], file_row['file_hash'], file_row['sections'],
            file_row['relative_path'], file_row['course_code'], file_row['course_name'],
            file_row['file_name'], file_row['extra_info'], file_row['url'],
            created_at, update_time
        ))

        stats["files"] += 1

        # Insert chunks for this file
        chunks = course_conn.execute("SELECT * FROM chunks WHERE file_uuid = ?", (file_uuid,)).fetchall()
        for chunk_row in chunks:
            collective_conn.execute("""
                INSERT OR REPLACE INTO chunks (chunk_uuid, file_uuid, idx, text, title, url,
                                             file_path, reference_path, course_name, course_code, chunk_index)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                chunk_row['chunk_uuid'], chunk_row['file_uuid'], chunk_row['idx'],
                chunk_row['text'], chunk_row['title'], chunk_row['url'],
                chunk_row['file_path'], chunk_row['reference_path'], chunk_row['course_name'],
                chunk_row['course_code'], chunk_row['chunk_index']
            ))
            stats["chunks"] += 1

        # Insert problems for this file
        problems = course_conn.execute("SELECT * FROM problem WHERE file_uuid = ?", (file_uuid,)).fetchall()
        for problem_row in problems:
            collective_conn.execute("""
                INSERT OR REPLACE INTO problem (uuid, file_uuid, problem_index, problem_id, problem_content,
                                              question_id, question, choices, answer, explanation, question_type)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                problem_row['uuid'], problem_row['file_uuid'], problem_row['problem_index'],
                problem_row['problem_id'], problem_row['problem_content'], problem_row['question_id'],
                problem_row['question'], problem_row['choices'], problem_row['answer'],
                problem_row['explanation'], problem_row['question_type']
            ))
            stats["problems"] += 1

    return stats

def merge_all_course_databases_in_directory(
    course_db_directory: str,
    collective_db_path: str,
    db_pattern: str = "*_metadata.db"
) -> Dict[str, int]:
    """Convenience function to merge all course databases in a directory into collective database.

    Args:
        course_db_directory: Directory containing course database files
        collective_db_path: Path to the collective/master database
        db_pattern: Pattern to match course database files (default: "*_metadata.db")

    Returns:
        Dictionary with merge statistics
    """

    course_db_dir = Path(course_db_directory)
    if not course_db_dir.exists():
        raise FileNotFoundError(f"Course database directory not found: {course_db_dir}")

    # Find all matching database files
    course_db_files = list(course_db_dir.glob(db_pattern))
    if not course_db_files:
        logging.warning(f"No course database files found matching pattern '{db_pattern}' in {course_db_dir}")
        return {}

    logging.info(f"Found {len(course_db_files)} course databases to merge")
    for db_file in course_db_files:
        logging.info(f"  - {db_file.name}")

    # Convert to string paths
    course_db_paths = [str(db_file) for db_file in course_db_files]

    return merge_course_databases_into_collective(
        course_db_paths=course_db_paths,
        collective_db_path=collective_db_path,
    )

def merge_databases_by_list(
    course_db_paths: List[str],
    collective_db_path: str
) -> Dict[str, int]:
    """Merge a list of course databases into collective database based on course codes.

    Args:
        course_db_paths: List of paths to individual course database files
        collective_db_path: Path to the collective/master database

    Returns:
        Dictionary with merge statistics
    """
    if not course_db_paths:
        logging.warning("No course database paths provided")
        return {}

    # Validate all database paths exist
    valid_paths = []
    for db_path in course_db_paths:
        db_path_obj = Path(db_path)
        if db_path_obj.exists():
            valid_paths.append(str(db_path))
        else:
            logging.warning(f"Course database not found: {db_path}")

    if not valid_paths:
        logging.warning("No valid course database paths found")
        return {}

    logging.info(f"Merging {len(valid_paths)} course databases into collective database")
    for db_path in valid_paths:
        logging.info(f"  - {Path(db_path).name}")

    return merge_course_databases_into_collective(
        course_db_paths=valid_paths,
        collective_db_path=collective_db_path,
    )
def split_course_from_collective(
    collective_db_path: str,
    course_code: str,
    output_course_db_path: str
) -> Dict[str, int]:
    """Split a specific course's data from collective database into individual course database.

    Args:
        collective_db_path: Path to the collective database
        course_code: Course code to extract
        output_course_db_path: Path for the new course database

    Returns:
        Dictionary with split statistics (files, chunks, problems extracted)
    """
    collective_db_path = Path(collective_db_path)
    output_course_db_path = Path(output_course_db_path)

    if not collective_db_path.exists():
        raise FileNotFoundError(f"Collective database not found: {collective_db_path}")

    # Ensure output directory exists
    output_course_db_path.parent.mkdir(parents=True, exist_ok=True)

    # Connect to collective database
    collective_conn = sqlite3.connect(str(collective_db_path))
    collective_conn.row_factory = sqlite3.Row

    # Connect to/create course database
    course_conn = sqlite3.connect(str(output_course_db_path))
    course_conn.row_factory = sqlite3.Row

    # Initialize schema in course database
    course_conn.executescript(SQL_INIT)

    split_stats = {"files": 0, "chunks": 0, "problems": 0}

    try:
        logging.info(f"Splitting course '{course_code}' from collective database")

        # Get all files for the specified course
        files = collective_conn.execute(
            "SELECT * FROM file WHERE course_code = ?", (course_code,)
        ).fetchall()

        if not files:
            logging.warning(f"No files found for course '{course_code}' in collective database")
            return split_stats

        logging.info(f"Found {len(files)} files for course '{course_code}'")

        # Copy files and related data
        for file_row in files:
            file_uuid = file_row['uuid']

            # Insert file into course database
            course_conn.execute("""
                INSERT OR REPLACE INTO file (uuid, file_hash, sections, relative_path,
                                           course_code, course_name, file_name, extra_info, url)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                file_row['uuid'], file_row['file_hash'], file_row['sections'],
                file_row['relative_path'], file_row['course_code'], file_row['course_name'],
                file_row['file_name'], file_row['extra_info'], file_row['url']
            ))
            split_stats["files"] += 1

            # Copy chunks for this file
            chunks = collective_conn.execute(
                "SELECT * FROM chunks WHERE file_uuid = ?", (file_uuid,)
            ).fetchall()

            for chunk_row in chunks:
                course_conn.execute("""
                    INSERT OR REPLACE INTO chunks (chunk_uuid, file_uuid, idx, text, title, url,
                                                 file_path, reference_path, course_name, course_code, chunk_index)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    chunk_row['chunk_uuid'], chunk_row['file_uuid'], chunk_row['idx'],
                    chunk_row['text'], chunk_row['title'], chunk_row['url'],
                    chunk_row['file_path'], chunk_row['reference_path'], chunk_row['course_name'],
                    chunk_row['course_code'], chunk_row['chunk_index']
                ))
                split_stats["chunks"] += 1

            # Copy problems for this file
            problems = collective_conn.execute(
                "SELECT * FROM problem WHERE file_uuid = ?", (file_uuid,)
            ).fetchall()

            for problem_row in problems:
                course_conn.execute("""
                    INSERT OR REPLACE INTO problem (uuid, file_uuid, problem_index, problem_id, problem_content,
                                                  question_id, question, choices, answer, explanation, question_type)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    problem_row['uuid'], problem_row['file_uuid'], problem_row['problem_index'],
                    problem_row['problem_id'], problem_row['problem_content'], problem_row['question_id'],
                    problem_row['question'], problem_row['choices'], problem_row['answer'],
                    problem_row['explanation'], problem_row['question_type']
                ))
                split_stats["problems"] += 1

        course_conn.commit()
        logging.info(f"Successfully split course '{course_code}' from collective database: {split_stats}")

    except Exception as e:
        course_conn.rollback()
        logging.error(f"Error during course split: {str(e)}")
        raise
    finally:
        collective_conn.close()
        course_conn.close()

    return split_stats

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Example usage: merge specific databases by providing a list of paths
    merge_stats = merge_databases_by_list(
        course_db_paths=[
            "/home/bot/bot/yk/YK_final/courses_out/db/CS 294-137_metadata.db",
            "/home/bot/bot/yk/YK_final/courses_out/db/CS 61A_metadata.db",
            "/home/bot/bot/yk/YK_final/courses_out/db/Berkeley_metadata.db",
            "/home/bot/bot/yk/YK_final/courses_out/db/ROAR Academy_metadata.db"
        ],
        collective_db_path="/home/bot/bot/yk/YK_final/course_yaml/collective_metadata.db"
    )
    logging.info(f"Merge statistics: {merge_stats}")
