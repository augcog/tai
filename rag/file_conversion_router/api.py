"""Public API for the file conversion router module."""

from pathlib import Path
from typing import Union, List, Dict
import yaml
import logging
from datetime import datetime
import sqlite3
import shutil
from pathlib import Path

from file_conversion_router.services.directory_service import process_folder

def load_yaml(file_path):
    """Load YAML configuration file."""
    with open(file_path, "r") as file:
        return yaml.safe_load(file)

def save_yaml(data, file_path):
    """Save data to YAML configuration file."""
    with open(file_path, "w") as file:
        yaml.safe_dump(data, file, default_flow_style=False, indent=2)
def convert_directory(input_config) -> None:
    """Convert all supported files in the given directory to Markdown format, to the specified output directory.

    Current supported file types:
    1. PDF
    2. Markdown (To clarify, this markdown includes additional tree structure of original markdown file)
    """
    data = load_yaml(input_config)
    input_dir = data["input_dir"]
    output_dir = data["output_dir"]
    course_name = data["course_name"]
    course_id = data["course_code"]
    log_dir = data.get("log_folder", None)
    db_path = data.get("db_path", None)
    process_folder(
        input_dir,
        output_dir,
        course_name,
        course_id,
        log_dir=log_dir,
        db_path=db_path,
    )

def process_courses_from_master_config(master_config_path: str = None) -> None:
    """Process all courses marked for update in the master configuration file.

    Args:
        master_config_path: Path to the master configuration file.
                          Defaults to courses_master_config.yaml in the same directory.
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent/"configs"/ "courses_master_config.yaml"

    if not Path(master_config_path).exists():
        raise FileNotFoundError(f"Master config file not found: {master_config_path}")

    master_config = load_yaml(master_config_path)
    courses_to_process = []

    # Find courses that need updating
    for course_name, course_info in master_config.get("courses", {}).items():
        if course_info.get("needs_update", False) and course_info.get("enabled", True):
            config_path = course_info.get("config_path")
            if config_path and Path(config_path).exists():
                courses_to_process.append((course_name, config_path))
                logging.info(f"Course '{course_name}' marked for processing")
            else:
                logging.warning(f"Config file not found for course '{course_name}': {config_path}")

    if not courses_to_process:
        logging.info("No courses marked for update")
        return

    logging.info(f"Processing {len(courses_to_process)} courses...")

    # Process each course
    processed_courses = []
    for course_name, config_path in courses_to_process:
        try:
            logging.info(f"Starting processing for course: {course_name}")
            convert_directory(config_path)
            processed_courses.append(course_name)
            logging.info(f"Successfully processed course: {course_name}")
        except Exception as e:
            logging.error(f"Failed to process course '{course_name}': {str(e)}")
            continue

    # Update master config to mark processed courses as completed
    if processed_courses:
        update_master_config_status(master_config_path, processed_courses)
        logging.info(f"Updated master config for processed courses: {processed_courses}")

def update_master_config_status(master_config_path: str, processed_courses: List[str]) -> None:
    """Update the master configuration file to mark courses as processed.

    Args:
        master_config_path: Path to the master configuration file
        processed_courses: List of course names that were successfully processed
    """
    master_config = load_yaml(master_config_path)
    current_time = datetime.now().minute

    for course_name in processed_courses:
        if course_name in master_config.get("courses", {}):
            master_config["courses"][course_name]["needs_update"] = False
            master_config["courses"][course_name]["last_updated"] = current_time

    save_yaml(master_config, master_config_path)

def get_courses_needing_update(master_config_path: str = None) -> List[Dict]:
    """Get list of courses that need updating.

    Args:
        master_config_path: Path to the master configuration file

    Returns:
        List of dictionaries containing course information for courses needing update
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent /'configs'/ "courses_master_config.yaml"

    master_config = load_yaml(master_config_path)
    courses_needing_update = []

    for course_name, course_info in master_config.get("courses", {}).items():
        if course_info.get("needs_update", False) and course_info.get("enabled", True):
            courses_needing_update.append({
                "name": course_name,
                "config_path": course_info.get("config_path"),
                "last_updated": course_info.get("last_updated")
            })

    return courses_needing_update

def mark_course_for_update(course_name: str, master_config_path: str = None) -> bool:
    """Mark a specific course for update.

    Args:
        course_name: Name of the course to mark for update
        master_config_path: Path to the master configuration file

    Returns:
        True if successful, False if course not found
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent / "courses_master_config.yaml"

    master_config = load_yaml(master_config_path)

    if course_name in master_config.get("courses", {}):
        master_config["courses"][course_name]["needs_update"] = True
        save_yaml(master_config, master_config_path)
        logging.info(f"Marked course '{course_name}' for update")
        return True
    else:
        logging.warning(f"Course '{course_name}' not found in master config")
        return False

def merge_course_databases_into_collective(course_db_paths: List[str],
                                         collective_db_path: str,
                                         ) -> Dict[str, int]:
    """Merge multiple small course databases into a collective database.

    Args:
        course_db_paths: List of paths to individual course database files
        collective_db_path: Path to the collective/master database
        backup_collective: Whether to backup the collective database before merging

    Returns:
        Dictionary with merge statistics (files, chunks, problems merged per course)
    """

    collective_db_path = Path(collective_db_path)

    # Ensure collective database exists and has proper schema
    collective_db_path.parent.mkdir(parents=True, exist_ok=True)
    collective_conn = sqlite3.connect(str(collective_db_path))
    collective_conn.row_factory = sqlite3.Row

    # Initialize schema in collective database
    from file_conversion_router.services.directory_service import SQL_INIT
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

def _merge_single_course_db(course_conn: sqlite3.Connection,
                          collective_conn: sqlite3.Connection,
                          course_db_name: str) -> Dict[str, int]:
    """Merge a single course database into the collective database.

    Returns statistics about the merge operation.
    """
    stats = {"files": 0, "chunks": 0, "problems": 0, "conflicts": 0}

    # Get all files from course database
    files = course_conn.execute("SELECT * FROM file").fetchall()

    for file_row in files:
        file_uuid = file_row['uuid']
        file_hash = file_row['file_hash']

        # Check if file already exists in collective database
        existing_file = collective_conn.execute(
            "SELECT uuid, course_code, course_name FROM file WHERE file_hash = ?",
            (file_hash,)
        ).fetchone()

        if existing_file:
            existing_uuid = existing_file['uuid']
            if existing_uuid != file_uuid:
                # Same file content but different UUID - this is a conflict
                logging.warning(f"File hash conflict: {file_hash} exists with different UUID in collective DB")
                logging.warning(f"  Course DB: {file_uuid} from {course_db_name}")
                logging.warning(f"  Collective DB: {existing_uuid} from course {existing_file['course_code']}")
                stats["conflicts"] += 1

                # Use the existing UUID and skip this file's chunks/problems
                continue
            else:
                # Same file, same UUID - update file metadata if needed
                collective_conn.execute("""
                    INSERT OR REPLACE INTO file (uuid, file_hash, sections, relative_path,
                                               course_code, course_name, file_name, extra_info, url)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    file_row['uuid'], file_row['file_hash'], file_row['sections'],
                    file_row['relative_path'], file_row['course_code'], file_row['course_name'],
                    file_row['file_name'], file_row['extra_info'], file_row['url']
                ))
        else:
            # New file - insert it
            collective_conn.execute("""
                INSERT OR REPLACE INTO file (uuid, file_hash, sections, relative_path,
                                           course_code, course_name, file_name, extra_info, url)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                file_row['uuid'], file_row['file_hash'], file_row['sections'],
                file_row['relative_path'], file_row['course_code'], file_row['course_name'],
                file_row['file_name'], file_row['extra_info'], file_row['url']
            ))

        stats["files"] += 1

        # Merge chunks for this file
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

        # Merge problems for this file
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

def merge_all_course_databases_in_directory(course_db_directory: str,
                                          collective_db_path: str,
                                          db_pattern: str = "*_metadata.db") -> Dict[str, int]:
    """Convenience function to merge all course databases in a directory into collective database.

    Args:
        course_db_directory: Directory containing course database files
        collective_db_path: Path to the collective/master database
        db_pattern: Pattern to match course database files (default: "*_metadata.db")

    Returns:
        Dictionary with merge statistics
    """
    from pathlib import Path
    import glob

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

# New collective processing approach:
if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Process all courses marked for update
    # process_courses_from_master_config()

    # Example: Merge course databases after processing
    # After processing individual courses, you can merge them into a collective database:
    #
    # merge_stats = merge_all_course_databases_in_directory(
    #     course_db_directory="/path/to/course/databases",
    #     collective_db_path="/path/to/collective_metadata.db",
    #     db_pattern="*_metadata.db"  # Match files like CS61A_metadata.db
    # )
    # print(f"Merge completed: {merge_stats}")
    #
    # Or merge specific databases:
    merge_stats = merge_course_databases_into_collective(
        course_db_paths=[
            "/home/bot/bot/yk/YK_final/courses_out/CS_294-137/CS_294-137_metadata.db",
            "/path/to/CS294_metadata.db"
        ],
        collective_db_path="/home/bot/bot/yk/YK_final/course_yaml/metadata.db"
    )
    print(f"Merge completed: {merge_stats}")