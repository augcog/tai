import sqlite3
import os
from pathlib import Path
from typing import List, Dict, Optional
import logging
import shutil

def split_collective_db_by_course(collective_db_path: str,
                                 output_dir: str = None,
                                 course_mapping: Dict[str, str] = None) -> Dict[str, str]:
    """
    Split the collective metadata database into separate databases by course code.

    Args:
        collective_db_path: Path to the collective metadata.db file
        output_dir: Directory to save the split databases (defaults to same dir as collective_db)
        course_mapping: Optional mapping of course_code -> custom_db_name

    Returns:
        Dictionary mapping course_code -> created_db_path
    """
    collective_db_path = Path(collective_db_path)
    if not collective_db_path.exists():
        raise FileNotFoundError(f"Collective database not found: {collective_db_path}")

    if output_dir is None:
        output_dir = collective_db_path.parent
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Connect to the collective database
    collective_conn = sqlite3.connect(str(collective_db_path))
    collective_conn.row_factory = sqlite3.Row

    try:
        # Get all distinct course codes
        course_codes = collective_conn.execute(
            "SELECT DISTINCT course_code FROM file WHERE course_code IS NOT NULL AND course_code != ''"
        ).fetchall()

        created_databases = {}

        for row in course_codes:
            course_code = row[0]
            if not course_code:
                continue

            # Determine database name
            if course_mapping and course_code in course_mapping:
                db_name = course_mapping[course_code]
            else:
                # Create safe filename from course code
                safe_course_code = course_code.replace(" ", "_").replace("/", "_")
                db_name = f"{safe_course_code}_metadata.db"

            db_path = output_dir / db_name

            logging.info(f"Creating database for course '{course_code}': {db_path}")

            # Create new database for this course
            course_conn = sqlite3.connect(str(db_path))
            course_conn.row_factory = sqlite3.Row

            try:
                # Create the same schema as the original database
                create_schema(course_conn)

                # Copy data for this course
                copy_course_data(collective_conn, course_conn, course_code)

                course_conn.commit()
                created_databases[course_code] = str(db_path)

                logging.info(f"Successfully created database for course '{course_code}'")

            except Exception as e:
                logging.error(f"Error creating database for course '{course_code}': {e}")
                course_conn.close()
                if db_path.exists():
                    db_path.unlink()  # Remove incomplete database
                raise
            finally:
                course_conn.close()

        logging.info(f"Successfully split collective database into {len(created_databases)} course databases")
        return created_databases

    finally:
        collective_conn.close()

def create_schema(conn: sqlite3.Connection) -> None:
    """
    Create the database schema in the new course database.
    """
    schema_sql = """
    PRAGMA foreign_keys=ON;
    PRAGMA journal_mode=WAL;
    PRAGMA synchronous=NORMAL;

    CREATE TABLE IF NOT EXISTS file (
        uuid         TEXT PRIMARY KEY,
        file_hash    TEXT NOT NULL UNIQUE,
        sections     TEXT,
        relative_path TEXT,
        course_code  TEXT,
        course_name  TEXT,
        file_name    TEXT,
        extra_info   TEXT,
        url          TEXT,
        vector       BLOB
    );

    CREATE TABLE IF NOT EXISTS chunks (
        chunk_uuid     TEXT PRIMARY KEY,
        file_uuid      TEXT NOT NULL,
        idx            INTEGER NOT NULL,
        text           TEXT NOT NULL,
        title          TEXT,
        url            TEXT,
        file_path      TEXT,
        reference_path TEXT,
        course_name    TEXT,
        course_code    TEXT,
        chunk_index    INTEGER,
        vector         BLOB,
        FOREIGN KEY (file_uuid) REFERENCES file(uuid) ON DELETE CASCADE
    );

    CREATE TABLE IF NOT EXISTS problem (
        uuid            TEXT PRIMARY KEY,
        file_uuid       TEXT NOT NULL,
        problem_index   INTEGER,
        problem_id      TEXT,
        problem_content TEXT,
        question_id     TEXT,
        question        TEXT,
        choices         TEXT,
        answer          TEXT,
        explanation     TEXT,
        question_type   TEXT DEFAULT 'regular',
        FOREIGN KEY (file_uuid) REFERENCES file(uuid) ON DELETE CASCADE
    );
    """

    conn.executescript(schema_sql)

def copy_course_data(source_conn: sqlite3.Connection,
                    dest_conn: sqlite3.Connection,
                    course_code: str) -> None:
    """
    Copy all data for a specific course from source to destination database.
    """
    # Get all files for this course
    files = source_conn.execute(
        "SELECT * FROM file WHERE course_code = ?", (course_code,)
    ).fetchall()

    if not files:
        logging.warning(f"No files found for course '{course_code}'")
        return

    file_uuids = [file['uuid'] for file in files]

    logging.info(f"Found {len(files)} files for course '{course_code}'")

    # Copy files
    for file_row in files:
        columns = list(file_row.keys())
        placeholders = ','.join(['?' for _ in columns])
        values = [file_row[col] for col in columns]

        dest_conn.execute(
            f"INSERT OR REPLACE INTO file ({','.join(columns)}) VALUES ({placeholders})",
            values
        )

    # Copy chunks for these files
    chunk_count = 0
    for file_uuid in file_uuids:
        chunks = source_conn.execute(
            "SELECT * FROM chunks WHERE file_uuid = ?", (file_uuid,)
        ).fetchall()

        for chunk_row in chunks:
            columns = list(chunk_row.keys())
            placeholders = ','.join(['?' for _ in columns])
            values = [chunk_row[col] for col in columns]

            dest_conn.execute(
                f"INSERT OR REPLACE INTO chunks ({','.join(columns)}) VALUES ({placeholders})",
                values
            )
            chunk_count += 1

    # Copy problems for these files
    problem_count = 0
    for file_uuid in file_uuids:
        problems = source_conn.execute(
            "SELECT * FROM problem WHERE file_uuid = ?", (file_uuid,)
        ).fetchall()

        for problem_row in problems:
            columns = list(problem_row.keys())
            placeholders = ','.join(['?' for _ in columns])
            values = [problem_row[col] for col in columns]

            dest_conn.execute(
                f"INSERT OR REPLACE INTO problem ({','.join(columns)}) VALUES ({placeholders})",
                values
            )
            problem_count += 1

    logging.info(f"Copied for course '{course_code}': {len(files)} files, {chunk_count} chunks, {problem_count} problems")

def get_course_db_stats(db_path: str) -> Dict[str, int]:
    """
    Get statistics for a course database.

    Args:
        db_path: Path to the course database

    Returns:
        Dictionary with table counts
    """
    conn = sqlite3.connect(db_path)
    try:
        stats = {}
        tables = ['file', 'chunks', 'problem']

        for table in tables:
            try:
                count = conn.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]
                stats[table] = count
            except sqlite3.OperationalError:
                stats[table] = 0  # Table might not exist

        return stats
    finally:
        conn.close()
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    # Example usage
    collective_db = "/home/bot/bot/yk/YK_final/courses/metadata (1).db"
    output_directory = "/home/bot/bot/yk/YK_final/test_folder"
    course_name_mapping = {
        "CS 61A": "CS 61A_metadata.db",
        "CS 294-137": "CS 294-137_metadata.db",
        "ROAR Academy": "ROAR Academy_metadata.db"
    }

    try:
        created_dbs = split_collective_db_by_course(
            collective_db_path=collective_db,
            output_dir=output_directory,
            course_mapping=course_name_mapping
        )

        for course_code, db_path in created_dbs.items():
            stats = get_course_db_stats(db_path)
            logging.info(f"Stats for course '{course_code}': {stats}")

    except Exception as e:
        logging.error(f"Error during database splitting: {e}")