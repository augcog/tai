"""Database merging utilities for combining course databases into collective databases."""

import logging
import sqlite3
from pathlib import Path
from typing import List, Dict, Optional


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


def _update_uuid_in_course_database(
    course_conn: sqlite3.Connection,
    old_uuid: str,
    new_uuid: str
) -> None:
    """Update UUID in all tables of the course database.

    Args:
        course_conn: Connection to the course database
        old_uuid: The current UUID to be replaced
        new_uuid: The new UUID to use
    """
    try:
        # Update file table
        course_conn.execute(
            "UPDATE file SET uuid = ? WHERE uuid = ?",
            (new_uuid, old_uuid)
        )

        # Update chunks table - file_uuid column
        course_conn.execute(
            "UPDATE chunks SET file_uuid = ? WHERE file_uuid = ?",
            (new_uuid, old_uuid)
        )

        # Update problem table - file_uuid column
        course_conn.execute(
            "UPDATE problem SET file_uuid = ? WHERE file_uuid = ?",
            (new_uuid, old_uuid)
        )

        # Commit the changes to the course database
        course_conn.commit()

        logging.info(f"Successfully updated UUID from {old_uuid} to {new_uuid} in course database")

    except Exception as e:
        course_conn.rollback()
        logging.error(f"Failed to update UUID in course database: {str(e)}")
        raise


def clean_duplicate_problems(db_path: str) -> Dict[str, int]:
    """Clean all duplicate problems from a database.

    Removes duplicates based on multiple criteria:
    1. Identical problem_content and question across all files (global duplicates)
    2. Identical problem_id within the same file
    3. Empty or meaningless problems

    Args:
        db_path: Path to the database file

    Returns:
        Dictionary with cleanup statistics
    """
    db_path = Path(db_path)

    if not db_path.exists():
        raise FileNotFoundError(f"Database not found: {db_path}")

    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row

    cleanup_stats = {
        "total_problems_before": 0,
        "total_problems_after": 0,
        "duplicates_removed": 0,
        "empty_problems_removed": 0,
        "duplicate_groups_found": 0
    }

    try:
        # Get initial count
        cleanup_stats["total_problems_before"] = conn.execute("SELECT COUNT(*) FROM problem").fetchone()[0]
        logging.info(f"Starting cleanup of {cleanup_stats['total_problems_before']} problems in {db_path.name}")

        # Step 1: Remove empty or meaningless problems
        empty_problems = conn.execute("""
            SELECT uuid FROM problem
            WHERE (problem_content IS NULL OR TRIM(problem_content) = '' OR problem_content = '...')
            AND (question IS NULL OR TRIM(question) = '' OR question = '...')
            AND (problem_id IS NULL OR TRIM(problem_id) = '')
        """).fetchall()

        if empty_problems:
            empty_uuids = [row['uuid'] for row in empty_problems]
            cleanup_stats["empty_problems_removed"] = len(empty_uuids)
            placeholders = ",".join("?" * len(empty_uuids))
            conn.execute(f"DELETE FROM problem WHERE uuid IN ({placeholders})", empty_uuids)
            logging.info(f"Removed {cleanup_stats['empty_problems_removed']} empty problems")

        # Step 2: Find global duplicates based on content (across all files)
        duplicates_query = """
        SELECT uuid,
               ROW_NUMBER() OVER (
                   PARTITION BY COALESCE(TRIM(problem_content), ''),
                                COALESCE(TRIM(question), '')
                   ORDER BY problem_index ASC, uuid ASC
               ) as row_num
        FROM problem
        WHERE NOT (
            (problem_content IS NULL OR TRIM(problem_content) = '' OR problem_content = '...')
            AND (question IS NULL OR TRIM(question) = '' OR question = '...')
        )
        """

        duplicates = conn.execute(duplicates_query).fetchall()
        duplicate_uuids = [row['uuid'] for row in duplicates if row['row_num'] > 1]

        if duplicate_uuids:
            content_duplicates = len(duplicate_uuids)

            # Count duplicate groups
            duplicate_groups_query = """
            SELECT COUNT(*) as groups FROM (
                SELECT COALESCE(TRIM(problem_content), ''),
                       COALESCE(TRIM(question), ''),
                       COUNT(*) as cnt
                FROM problem
                WHERE NOT (
                    (problem_content IS NULL OR TRIM(problem_content) = '' OR problem_content = '...')
                    AND (question IS NULL OR TRIM(question) = '' OR question = '...')
                )
                GROUP BY COALESCE(TRIM(problem_content), ''),
                         COALESCE(TRIM(question), '')
                HAVING COUNT(*) > 1
            )
            """
            cleanup_stats["duplicate_groups_found"] = conn.execute(duplicate_groups_query).fetchone()[0]

            # Delete duplicates
            placeholders = ",".join("?" * len(duplicate_uuids))
            conn.execute(f"DELETE FROM problem WHERE uuid IN ({placeholders})", duplicate_uuids)

            cleanup_stats["duplicates_removed"] += content_duplicates
            logging.info(f"Removed {content_duplicates} content duplicates from {cleanup_stats['duplicate_groups_found']} duplicate groups")

        # Step 3: Handle problems with identical problem_id within the same file (keep most recent)
        problem_id_duplicates_query = """
        SELECT uuid,
               ROW_NUMBER() OVER (
                   PARTITION BY file_uuid, TRIM(problem_id)
                   ORDER BY problem_index DESC, uuid DESC
               ) as row_num
        FROM problem
        WHERE problem_id IS NOT NULL AND TRIM(problem_id) != ''
        """

        problem_id_duplicates = conn.execute(problem_id_duplicates_query).fetchall()
        problem_id_duplicate_uuids = [row['uuid'] for row in problem_id_duplicates if row['row_num'] > 1]

        if problem_id_duplicate_uuids:
            additional_removed = len(problem_id_duplicate_uuids)
            placeholders = ",".join("?" * len(problem_id_duplicate_uuids))
            conn.execute(f"DELETE FROM problem WHERE uuid IN ({placeholders})", problem_id_duplicate_uuids)

            cleanup_stats["duplicates_removed"] += additional_removed
            logging.info(f"Removed additional {additional_removed} problems with duplicate problem_ids within same file")

        conn.commit()

        # Get final count
        cleanup_stats["total_problems_after"] = conn.execute("SELECT COUNT(*) FROM problem").fetchone()[0]

        logging.info(f"Cleanup completed: {cleanup_stats['total_problems_before']} → {cleanup_stats['total_problems_after']} problems")
        logging.info(f"Total removed: {cleanup_stats['duplicates_removed'] + cleanup_stats['empty_problems_removed']} problems")

    except Exception as e:
        conn.rollback()
        logging.error(f"Error during cleanup: {str(e)}")
        raise
    finally:
        conn.close()

    return cleanup_stats


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

def validate_comprehensive_questions(db_path: str) -> Dict[str, any]:
    """Validate comprehensive questions and identify data integrity issues.

    For comprehensive questions, problem_content and question fields should be identical.
    This function identifies questions with missing or mismatched content.

    Args:
        db_path: Path to the database file

    Returns:
        Dictionary with validation statistics and problem UUIDs
    """
    db_path = Path(db_path)

    if not db_path.exists():
        raise FileNotFoundError(f"Database not found: {db_path}")

    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row

    validation_stats = {
        "total_comprehensive": 0,
        "valid_questions": 0,
        "empty_question_field": 0,
        "empty_problem_content_field": 0,
        "both_fields_empty": 0,
        "mismatched_content": 0,
        "issues_found": []
    }

    try:
        # Get all comprehensive questions
        comprehensive_questions = conn.execute("""
            SELECT uuid, problem_content, question
            FROM problem
            WHERE question_type = 'comprehensive'
        """).fetchall()

        validation_stats["total_comprehensive"] = len(comprehensive_questions)

        for row in comprehensive_questions:
            uuid = row['uuid']
            problem_content = row['problem_content'] or ''
            question = row['question'] or ''

            problem_content = problem_content.strip()
            question = question.strip()

            # Check for various issues
            if not problem_content and not question:
                validation_stats["both_fields_empty"] += 1
                validation_stats["issues_found"].append({
                    "uuid": uuid,
                    "issue_type": "both_empty",
                    "problem_content": problem_content,
                    "question": question
                })
            elif not question:
                validation_stats["empty_question_field"] += 1
                validation_stats["issues_found"].append({
                    "uuid": uuid,
                    "issue_type": "empty_question",
                    "problem_content": problem_content,
                    "question": question
                })
            elif not problem_content:
                validation_stats["empty_problem_content_field"] += 1
                validation_stats["issues_found"].append({
                    "uuid": uuid,
                    "issue_type": "empty_problem_content",
                    "problem_content": problem_content,
                    "question": question
                })
            elif problem_content != question:
                validation_stats["mismatched_content"] += 1
                validation_stats["issues_found"].append({
                    "uuid": uuid,
                    "issue_type": "content_mismatch",
                    "problem_content": problem_content,
                    "question": question
                })
            else:
                validation_stats["valid_questions"] += 1

        total_issues = (validation_stats["empty_question_field"] +
                       validation_stats["empty_problem_content_field"] +
                       validation_stats["both_fields_empty"] +
                       validation_stats["mismatched_content"])

        logging.info(f"Comprehensive questions validation for {db_path.name}:")
        logging.info(f"  Total comprehensive questions: {validation_stats['total_comprehensive']}")
        logging.info(f"  Valid questions: {validation_stats['valid_questions']}")
        logging.info(f"  Issues found: {total_issues}")
        logging.info(f"    - Empty question field: {validation_stats['empty_question_field']}")
        logging.info(f"    - Empty problem_content field: {validation_stats['empty_problem_content_field']}")
        logging.info(f"    - Both fields empty: {validation_stats['both_fields_empty']}")
        logging.info(f"    - Content mismatch: {validation_stats['mismatched_content']}")

    except Exception as e:
        logging.error(f"Error during comprehensive questions validation: {str(e)}")
        raise
    finally:
        conn.close()

    return validation_stats


def fix_comprehensive_questions(db_path: str) -> Dict[str, int]:
    """Fix comprehensive questions with missing or inconsistent content.

    Repairs comprehensive questions by ensuring problem_content and question fields
    contain identical content. Uses the following repair strategy:
    1. If question is empty but problem_content exists: Copy problem_content to question
    2. If problem_content is empty but question exists: Copy question to problem_content
    3. If both have content but differ: Keep the longer/more complete version in both
    4. If both are empty: Log for manual review (no automatic fix)

    Args:
        db_path: Path to the database file

    Returns:
        Dictionary with repair statistics
    """
    db_path = Path(db_path)

    if not db_path.exists():
        raise FileNotFoundError(f"Database not found: {db_path}")

    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row

    repair_stats = {
        "questions_fixed": 0,
        "question_field_filled": 0,
        "problem_content_field_filled": 0,
        "content_standardized": 0,
        "manual_review_needed": 0,
        "manual_review_uuids": []
    }

    try:
        # First validate to get issues
        validation_results = validate_comprehensive_questions(db_path)

        if not validation_results["issues_found"]:
            logging.info("No comprehensive questions need fixing.")
            return repair_stats

        logging.info(f"Starting repair of {len(validation_results['issues_found'])} comprehensive questions...")

        for issue in validation_results["issues_found"]:
            uuid = issue["uuid"]
            problem_content = issue["problem_content"]
            question = issue["question"]
            issue_type = issue["issue_type"]

            new_problem_content = problem_content
            new_question = question
            fixed = False

            if issue_type == "empty_question":
                # Copy problem_content to question
                new_question = problem_content
                repair_stats["question_field_filled"] += 1
                fixed = True

            elif issue_type == "empty_problem_content":
                # Copy question to problem_content
                new_problem_content = question
                repair_stats["problem_content_field_filled"] += 1
                fixed = True

            elif issue_type == "content_mismatch":
                # Use the longer/more complete version for both fields
                if len(problem_content) >= len(question):
                    new_question = problem_content
                else:
                    new_problem_content = question
                repair_stats["content_standardized"] += 1
                fixed = True

            elif issue_type == "both_empty":
                # Cannot auto-fix, needs manual review
                repair_stats["manual_review_needed"] += 1
                repair_stats["manual_review_uuids"].append(uuid)
                logging.warning(f"Comprehensive question {uuid} has both fields empty - requires manual review")
                continue

            if fixed:
                # Update the database
                conn.execute("""
                    UPDATE problem
                    SET problem_content = ?, question = ?
                    WHERE uuid = ?
                """, (new_problem_content, new_question, uuid))

                repair_stats["questions_fixed"] += 1

        conn.commit()

        logging.info(f"Comprehensive questions repair completed:")
        logging.info(f"  Questions fixed: {repair_stats['questions_fixed']}")
        logging.info(f"  Question fields filled: {repair_stats['question_field_filled']}")
        logging.info(f"  Problem content fields filled: {repair_stats['problem_content_field_filled']}")
        logging.info(f"  Content standardized: {repair_stats['content_standardized']}")
        logging.info(f"  Manual review needed: {repair_stats['manual_review_needed']}")

    except Exception as e:
        conn.rollback()
        logging.error(f"Error during comprehensive questions repair: {str(e)}")
        raise
    finally:
        conn.close()

    return repair_stats


def verify_comprehensive_questions_fix(db_path: str) -> Dict[str, any]:
    """Verify that comprehensive questions have been properly fixed.

    Re-validates comprehensive questions after repair to ensure all issues
    have been resolved.

    Args:
        db_path: Path to the database file

    Returns:
        Dictionary with verification results
    """
    logging.info(f"Verifying comprehensive questions fix for {Path(db_path).name}...")

    validation_results = validate_comprehensive_questions(db_path)

    verification = {
        "total_comprehensive": validation_results["total_comprehensive"],
        "valid_questions": validation_results["valid_questions"],
        "remaining_issues": len(validation_results["issues_found"]),
        "fix_success_rate": 0.0,
        "verification_passed": False
    }

    if validation_results["total_comprehensive"] > 0:
        verification["fix_success_rate"] = (validation_results["valid_questions"] /
                                          validation_results["total_comprehensive"]) * 100

    # Consider fix successful if all questions are valid or only manual review cases remain
    manual_review_only = all(issue["issue_type"] == "both_empty"
                           for issue in validation_results["issues_found"])

    verification["verification_passed"] = (validation_results["valid_questions"] ==
                                         validation_results["total_comprehensive"] or
                                         manual_review_only)

    logging.info(f"Verification results:")
    logging.info(f"  Success rate: {verification['fix_success_rate']:.1f}%")
    logging.info(f"  Verification passed: {verification['verification_passed']}")

    return verification


if __name__ == "__main__":
    import logging

    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Example usage: Merge single course database with proper transaction handling
    try:
        course_conn = sqlite3.connect("/home/bot/bot/yk/YK_final/courses_out/CS_294-137/CS_294-137_metadata.db")
        collective_conn = sqlite3.connect("/home/bot/bot/yk/YK_final/course_yaml/metadata.db")

        merge_stats = _merge_single_course_db(
            course_conn=course_conn,
            collective_conn=collective_conn,
            course_db_name="CS_294-137_metadata.db"
        )

        # IMPORTANT: Commit the transaction to persist changes
        collective_conn.commit()

        print(f"Merge completed: {merge_stats}")

        course_conn.close()
        collective_conn.close()

    except Exception as e:
        print(f"Merge failed: {e}")
        if 'collective_conn' in locals():
            collective_conn.rollback()
            collective_conn.close()
        if 'course_conn' in locals():
            course_conn.close()

    # Fix comprehensive questions
    try:
        print("\n" + "="*50)
        print("FIXING COMPREHENSIVE QUESTIONS")
        print("="*50)

        db_path = "/home/bot/bot/yk/YK_final/course_yaml/metadata.db"

        # Step 1: Validate current state
        validation_results = validate_comprehensive_questions(db_path)

        # Step 2: Fix issues
        if validation_results["issues_found"]:
            repair_results = fix_comprehensive_questions(db_path)
            print(f"Repair completed: {repair_results}")

            # Step 3: Verify fixes
            verification_results = verify_comprehensive_questions_fix(db_path)
            print(f"Verification: {verification_results}")
        else:
            print("No comprehensive questions issues found.")

    except Exception as e:
        print(f"Comprehensive questions fix failed: {e}")

    # Example: Clean duplicate problems
    # clean_duplicate_problems("/home/bot/bot/yk/YK_final/course_yaml/metadata.db")