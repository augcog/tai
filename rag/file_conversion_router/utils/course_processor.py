"""Course processing utilities for managing course updates and configurations."""

import logging
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Union, Any

from .yaml_utils import load_yaml, save_yaml
from file_conversion_router.services.directory_service import process_folder
from .database_merger import merge_databases_by_list
from file_conversion_router.embedding.embedding_create import embedding_create
from file_conversion_router.embedding.file_embedding_create import (
    embed_files_from_markdown,
)


def convert_directory(input_config: Union[str, Path], auto_embed: bool = True) -> None:
    """Convert all supported files in the given directory to Markdown format and optionally create embeddings.

    Current supported file types:
    1. PDF
    2. Markdown (To clarify, this markdown includes additional tree structure of original markdown file)

    Args:
        input_config: Path to the configuration YAML file
        auto_embed: Whether to automatically create embeddings after conversion (default: True)
    """
    data = load_yaml(str(input_config))
    input_dir = data["input_dir"]
    output_dir = data["output_dir"]
    course_name = data["course_name"]
    course_id = data["course_code"]
    log_dir = data.get("log_folder", None)
    db_path = data.get("db_path", None)

    # Process files to markdown
    process_folder(
        input_dir,
        output_dir,
        course_name,
        course_id,
        log_dir=log_dir,
        db_path=db_path,
    )

    # Auto-create embeddings if enabled and database path exists
    if auto_embed and db_path:
        try:
            # Embed chunks
            logging.info(f"Auto-embedding chunks started for course: {course_name}")
            embedding_create(db_path, course_id)
            logging.info(f"Auto-embedding chunks completed for course: {course_name}")

            # Embed MD files
            logging.info(f"Auto-embedding MD files started for course: {course_name}")
            file_embed_results = embed_files_from_markdown(
                db_path=db_path,
                data_dir=output_dir,
                course_filter=course_id,
                force_recompute=False
            )
            logging.info(f"Auto-embedding MD files completed for course: {course_name}. "
                        f"Processed: {file_embed_results.get('processed', 0)}, "
                        f"Errors: {file_embed_results.get('errors', 0)}")
        except Exception as e:
            logging.error(f"Auto-embedding failed for course '{course_name}': {str(e)}")
            # Continue processing - embedding failure shouldn't stop the pipeline


def process_courses_from_master_config(master_config_path: Optional[str] = None, auto_embed: bool = True) -> None:
    """Process all courses marked for update in the master configuration file.

    Args:
        master_config_path: Path to the master configuration file.
                          Defaults to courses_master_config.yaml in configs directory.
        auto_embed: Whether to automatically create embeddings after each course conversion (default: True)
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent.parent / "configs" / "courses_master_config.yaml"

    if not Path(master_config_path).exists():
        raise FileNotFoundError(f"Master config file not found: {master_config_path}")

    master_config = load_yaml(str(master_config_path))
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
            convert_directory(config_path, auto_embed=auto_embed)
            processed_courses.append(course_name)
            logging.info(f"Successfully processed course: {course_name}")
        except Exception as e:
            logging.error(f"Failed to process course '{course_name}': {str(e)}")
            continue

    # Update master config to mark processed courses as completed
    if processed_courses:
        update_master_config_status(str(master_config_path), processed_courses)
        logging.info(f"Updated master config for processed courses: {processed_courses}")


def update_master_config_status(master_config_path: str, processed_courses: List[str]) -> None:
    """Update the master configuration file to mark courses as processed.

    Args:
        master_config_path: Path to the master configuration file
        processed_courses: List of course names that were successfully processed
    """
    master_config = load_yaml(master_config_path)
    current_time = datetime.now().isoformat()
    for course_name in processed_courses:
        if course_name in master_config.get("courses", {}):
            master_config["courses"][course_name]["needs_update"] = False
            master_config["courses"][course_name]["last_updated"] = current_time

    save_yaml(master_config, master_config_path)


def get_courses_needing_update(master_config_path: Optional[str] = None) -> List[Dict[str, Optional[str]]]:
    """Get list of courses that need updating.

    Args:
        master_config_path: Path to the master configuration file

    Returns:
        List of dictionaries containing course information for courses needing update
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent.parent / "configs" / "courses_master_config.yaml"

    master_config = load_yaml(str(master_config_path))
    courses_needing_update = []

    for course_name, course_info in master_config.get("courses", {}).items():
        if course_info.get("needs_update", False) and course_info.get("enabled", True):
            courses_needing_update.append({
                "name": course_name,
                "config_path": course_info.get("config_path"),
                "last_updated": course_info.get("last_updated")
            })

    return courses_needing_update


def mark_course_for_update(course_name: str, master_config_path: Optional[str] = None) -> bool:
    """Mark a specific course for update.

    Args:
        course_name: Name of the course to mark for update
        master_config_path: Path to the master configuration file

    Returns:
        True if successful, False if course not found
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent.parent / "configs" / "courses_master_config.yaml"

    master_config = load_yaml(str(master_config_path))

    if course_name in master_config.get("courses", {}):
        master_config["courses"][course_name]["needs_update"] = True
        save_yaml(master_config, str(master_config_path))
        logging.info(f"Marked course '{course_name}' for update")
        return True
    else:
        logging.warning(f"Course '{course_name}' not found in master config")
        return False


def merge_course_databases_from_master_config(
    master_config_path: Optional[str] = None,
    exclude_test: bool = True,
    check_embeddings: bool = True
) -> Dict[str, Any]:
    """Merge all course databases into collective database using paths from master config.

    Args:
        master_config_path: Path to the master configuration file.
                          Defaults to courses_master_config.yaml in configs directory.
        exclude_test: Whether to exclude test databases from merging (default: True)
        check_embeddings: Whether to check and report embedding statistics after merge (default: True)

    Returns:
        Dictionary with merge statistics and embedding information
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent.parent / "configs" / "courses_master_config.yaml"

    if not Path(master_config_path).exists():
        raise FileNotFoundError(f"Master config file not found: {master_config_path}")

    master_config = load_yaml(str(master_config_path))

    # Get collective database path from master config
    collective_db_path = master_config.get("global", {}).get("collective_db_path")
    if not collective_db_path:
        raise ValueError("collective_db_path not found in master config global settings")

    # Collect database paths from all enabled courses
    course_db_paths = []
    excluded_courses = []
    included_courses = []

    for course_name, course_info in master_config.get("courses", {}).items():
        # Skip test databases if exclude_test is True
        if exclude_test and course_name.lower() in ['test', 'testing', 'demo']:
            logging.info(f"Excluding test course from merge: {course_name}")
            excluded_courses.append(course_name)
            continue

        if not course_info.get("enabled", True):
            logging.info(f"Skipping disabled course: {course_name}")
            excluded_courses.append(course_name)
            continue

        config_path = course_info.get("config_path")
        if not config_path or not Path(config_path).exists():
            logging.warning(f"Config file not found for course '{course_name}': {config_path}")
            continue

        try:
            # Load course config to get database path
            course_config = load_yaml(str(config_path))
            db_path = course_config.get("db_path")

            if db_path and Path(db_path).exists():
                course_db_paths.append(str(db_path))
                included_courses.append(course_name)
                logging.info(f"Found database for course '{course_name}': {db_path}")
            else:
                logging.warning(f"Database not found for course '{course_name}': {db_path}")

        except Exception as e:
            logging.error(f"Error reading config for course '{course_name}': {str(e)}")
            continue

    if not course_db_paths:
        logging.warning("No course databases found to merge")
        return {
            "merge_stats": {},
            "excluded_courses": excluded_courses,
            "included_courses": included_courses,
            "embedding_stats": {}
        }

    logging.info(f"Merging {len(course_db_paths)} course databases into collective database")
    logging.info(f"Collective database path: {collective_db_path}")
    logging.info(f"Excluded courses: {excluded_courses}")
    logging.info(f"Included courses: {included_courses}")

    # === PRE-MERGE VALIDATION: Check and fix missing embeddings ===
    logging.info("")
    logging.info("=" * 60)
    logging.info("PRE-MERGE EMBEDDING VALIDATION")
    logging.info("=" * 60)

    from file_conversion_router.api import create_embeddings_for_course

    courses_with_missing_embeddings = []
    embedding_errors = []

    for idx, db_path in enumerate(course_db_paths):
        course_name = included_courses[idx] if idx < len(included_courses) else f"Course_{idx}"

        try:
            # Check embedding status for this course
            embed_status = check_embedding_status(db_path)

            files_pending = embed_status.get("files_pending", 0)
            chunks_pending = embed_status.get("chunks_pending", 0)
            total_files = embed_status.get("total_files", 0)
            total_chunks = embed_status.get("total_chunks", 0)

            logging.info(f"Checking {course_name}:")
            logging.info(f"  Files: {total_files - files_pending}/{total_files} embedded")
            logging.info(f"  Chunks: {total_chunks - chunks_pending}/{total_chunks} embedded")

            # If embeddings are missing, try to generate them
            if files_pending > 0 or chunks_pending > 0:
                courses_with_missing_embeddings.append(course_name)
                logging.warning(f"⚠️  {course_name} has missing embeddings!")
                logging.warning(f"   Files pending: {files_pending}, Chunks pending: {chunks_pending}")

                # Try to get course_code from database
                import sqlite3
                conn = sqlite3.connect(db_path)
                try:
                    # Get course_code from first file record
                    row = conn.execute("SELECT course_code FROM file LIMIT 1").fetchone()
                    if row:
                        course_code = row[0]
                        logging.info(f"   Attempting to generate missing embeddings for {course_code}...")

                        # Generate missing embeddings
                        create_embeddings_for_course(
                            db_path=db_path,
                            course_code=course_code,
                            data_dir=None,  # Will auto-infer
                            force_recompute=False  # Only generate missing ones
                        )

                        # Re-check status
                        updated_status = check_embedding_status(db_path)
                        files_now_pending = updated_status.get("files_pending", 0)
                        chunks_now_pending = updated_status.get("chunks_pending", 0)

                        if files_now_pending == 0 and chunks_now_pending == 0:
                            logging.info(f"   ✅ Successfully generated all missing embeddings for {course_name}")
                        else:
                            logging.warning(f"   ⚠️  Some embeddings still missing: files={files_now_pending}, chunks={chunks_now_pending}")
                    else:
                        logging.warning(f"   Could not determine course_code for {course_name}")
                        embedding_errors.append(course_name)
                finally:
                    conn.close()

        except Exception as e:
            logging.error(f"Error checking/fixing embeddings for {course_name}: {str(e)}")
            embedding_errors.append(course_name)
            continue

    if courses_with_missing_embeddings:
        logging.warning("")
        logging.warning(f"⚠️  {len(courses_with_missing_embeddings)} course(s) had missing embeddings: {courses_with_missing_embeddings}")
    else:
        logging.info("")
        logging.info("✅ All courses have complete embeddings")

    if embedding_errors:
        logging.error(f"❌ {len(embedding_errors)} course(s) had embedding errors: {embedding_errors}")

    logging.info("=" * 60)
    logging.info("")

    # Use merge_databases_by_list to perform the merge
    merge_stats = merge_databases_by_list(
        course_db_paths=course_db_paths,
        collective_db_path=collective_db_path
    )

    # Check embedding statistics and database integrity if requested
    embedding_stats = {}
    validation_results = {}
    if check_embeddings and Path(collective_db_path).exists():
        try:
            from file_conversion_router.embedding.file_embedding_create import check_embedding_status
            from file_conversion_router.utils.database_validator import DatabaseValidator

            # Get overall embedding statistics
            overall_stats = check_embedding_status(collective_db_path)

            # Get per-course statistics
            course_stats = {}
            for course_name in included_courses:
                course_stats[course_name] = check_embedding_status(collective_db_path, course_name)

            embedding_stats = {
                "overall": overall_stats,
                "by_course": course_stats
            }

            # Validate database integrity
            validator = DatabaseValidator(collective_db_path)
            validation_results = {
                "integrity": validator.check_database_integrity(),
                "embedding_completeness": validator.check_embedding_completeness()
            }

            # Log summary
            logging.info("=" * 60)
            logging.info("DATABASE MERGE SUMMARY")
            logging.info("=" * 60)

            # Log merge results
            logging.info(f"Included Courses: {included_courses}")
            logging.info(f"Excluded Courses: {excluded_courses}")

            # Log embedding statistics
            if "overall" in embedding_stats:
                total_files = embedding_stats["overall"].get("total_files", 0)
                files_embedded = embedding_stats["overall"].get("files_embedded", 0)
                chunks_embedded = embedding_stats["overall"].get("chunks_embedded", 0)
                total_chunks = embedding_stats["overall"].get("total_chunks", 0)

                logging.info("")
                logging.info("EMBEDDING STATISTICS:")
                logging.info(f"  Total Files: {total_files}")
                logging.info(f"  Files with Embeddings: {files_embedded} ({files_embedded/total_files*100:.1f}%)" if total_files > 0 else "  Files with Embeddings: 0 (0%)")
                logging.info(f"  Total Chunks: {total_chunks}")
                logging.info(f"  Chunks with Embeddings: {chunks_embedded} ({chunks_embedded/total_chunks*100:.1f}%)" if total_chunks > 0 else "  Chunks with Embeddings: 0 (0%)")

            # Log validation issues
            if validation_results.get("integrity"):
                integrity = validation_results["integrity"]
                if integrity.get("null_columns"):
                    logging.warning("")
                    logging.warning("⚠️  DATABASE INTEGRITY ISSUES DETECTED:")
                    logging.warning(f"  - {len(integrity['null_columns'])} columns are completely NULL")
                    for null_col in integrity['null_columns'][:5]:  # Show first 5
                        logging.warning(f"    • {null_col['table']}.{null_col['column']}")
                    if len(integrity['null_columns']) > 5:
                        logging.warning(f"    ... and {len(integrity['null_columns']) - 5} more")

                if integrity.get("empty_tables"):
                    logging.warning(f"  - {len(integrity['empty_tables'])} tables are empty: {', '.join(integrity['empty_tables'])}")

            logging.info("=" * 60)

        except Exception as e:
            logging.error(f"Error checking database status: {str(e)}")
            embedding_stats = {"error": str(e)}

    return {
        "merge_stats": merge_stats,
        "excluded_courses": excluded_courses,
        "included_courses": included_courses,
        "embedding_stats": embedding_stats,
        "validation": validation_results
    }