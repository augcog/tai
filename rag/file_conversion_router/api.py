#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Public API for the file conversion router module.

This module provides a high-level interface for document conversion, embedding generation,
and database management for the RAG pipeline.

This file can be run as a script OR imported as a library:
- Run as script: python -m file_conversion_router.api
- Import as library: from file_conversion_router.api import convert_directory
"""

import logging
from pathlib import Path
from typing import Dict, List, Optional, Union, Any

# Import from utility modules
from file_conversion_router.utils.yaml_utils import load_yaml, save_yaml
from file_conversion_router.utils.course_processor import (
    convert_directory as _convert_directory,
    process_courses_from_master_config as _process_courses_from_master_config,
    update_master_config_status,
    get_courses_needing_update,
    mark_course_for_update,
    merge_course_databases_from_master_config  # Now supports exclude_test and check_embeddings params
)
from file_conversion_router.utils.database_merger import (
    merge_course_databases_into_collective,
    merge_all_course_databases_in_directory,
    merge_databases_by_list
)
from file_conversion_router.embedding.embedding_create import embedding_create
from file_conversion_router.embedding.file_embedding_create import (
    embed_files_from_markdown,
    check_embedding_status
)

# Configure logging
logger = logging.getLogger(__name__)


# ========================
# Main Conversion Functions
# ========================

def convert_directory(
    input_config: Union[str, Path],
    auto_embed: bool = True
) -> Dict[str, Any]:
    """
    Convert all supported files in a directory to Markdown format and optionally create embeddings.

    This is the main entry point for processing a single course directory. It handles:
    1. File conversion to Markdown format
    2. Database creation/update with file metadata
    3. Optional embedding generation for both chunks and full files

    Args:
        input_config: Path to the configuration YAML file containing:
            - input_dir: Source directory with course materials
            - output_dir: Destination for converted Markdown files
            - course_name: Full name of the course
            - course_code: Short code for the course (e.g., "CS61A")
            - db_path: Path to SQLite database for metadata
            - log_folder: Optional directory for log files
        auto_embed: Whether to automatically create embeddings after conversion (default: True)

    Returns:
        Dictionary containing conversion statistics:
            - files_processed: Number of files successfully converted
            - files_failed: Number of files that failed conversion
            - embeddings_created: Number of embeddings created (if auto_embed=True)
            - total_time: Total processing time in seconds

    Example:
        >>> result = convert_directory("configs/CS61A_config.yaml", auto_embed=True)
        >>> print(f"Processed {result['files_processed']} files")
    """
    try:
        # Load configuration
        data = load_yaml(str(input_config))

        # Track statistics
        stats = {
            "files_processed": 0,
            "files_failed": 0,
            "embeddings_created": 0,
            "chunks_embedded": 0,
            "total_time": 0
        }

        # Perform conversion with the original function
        logger.info(f"Starting conversion for config: {input_config}")
        _convert_directory(input_config, auto_embed=auto_embed)

        # If embeddings were created, get statistics
        if auto_embed and data.get("db_path"):
            embed_stats = check_embedding_status(data["db_path"])
            stats["embeddings_created"] = embed_stats.get("total_embedded", 0)
            stats["chunks_embedded"] = embed_stats.get("chunks_embedded", 0)

        logger.info(f"Conversion completed. Stats: {stats}")
        return stats

    except Exception as e:
        logger.error(f"Error in convert_directory: {str(e)}")
        raise


def process_courses_from_master_config(
    master_config_path: Optional[str] = None,
    auto_embed: bool = True
) -> Dict[str, Any]:
    """
    Process all courses marked for update in the master configuration file.

    This function provides batch processing capabilities for multiple courses,
    automatically handling conversion, embedding, and status updates.

    Args:
        master_config_path: Path to the master configuration file.
                          Defaults to 'configs/courses_master_config.yaml'
        auto_embed: Whether to automatically create embeddings after each course conversion

    Returns:
        Dictionary containing processing statistics:
            - courses_processed: List of successfully processed course names
            - courses_failed: List of courses that failed processing
            - total_files: Total number of files processed
            - total_embeddings: Total number of embeddings created

    Example:
        >>> results = process_courses_from_master_config(auto_embed=True)
        >>> print(f"Processed courses: {results['courses_processed']}")
    """
    stats = {
        "courses_processed": [],
        "courses_failed": [],
        "total_files": 0,
        "total_embeddings": 0
    }

    try:
        # Use the original function
        _process_courses_from_master_config(master_config_path, auto_embed=auto_embed)

        # Get list of processed courses from master config
        if master_config_path is None:
            master_config_path = Path(__file__).parent / "configs" / "courses_master_config.yaml"

        master_config = load_yaml(str(master_config_path))
        for course_name, course_info in master_config.get("courses", {}).items():
            if not course_info.get("needs_update", True):
                stats["courses_processed"].append(course_name)

        logger.info(f"Batch processing completed. Stats: {stats}")
        return stats

    except Exception as e:
        logger.error(f"Error in process_courses_from_master_config: {str(e)}")
        raise


# ========================
# Embedding Functions
# ========================

def create_embeddings_for_course(
    db_path: str,
    course_code: str,
    data_dir: Optional[str] = None,
    force_recompute: bool = False
) -> Dict[str, int]:
    """
    Create embeddings for a specific course's converted content.

    This function generates embeddings for BOTH:
    1. Document chunks (chunks.vector) - for RAG retrieval
    2. Complete Markdown files (file.vector) - for semantic search

    Both types of embeddings are REQUIRED for proper RAG functionality.

    Args:
        db_path: Path to the course's SQLite database
        course_code: Course identifier (e.g., "CS61A")
        data_dir: Directory containing converted Markdown files.
                 If None, will attempt to infer from course config.
        force_recompute: Whether to regenerate existing embeddings

    Returns:
        Dictionary with embedding statistics:
            - chunks_embedded: Number of chunks with embeddings
            - files_embedded: Number of files with embeddings
            - errors: Number of errors encountered
            - skipped: Number of items skipped (already embedded)

    Example:
        >>> stats = create_embeddings_for_course(
        ...     db_path="data/CS61A_metadata.db",
        ...     course_code="CS61A",
        ...     force_recompute=False
        ... )
        >>> print(f"Created embeddings for {stats['chunks_embedded']} chunks")
    """
    stats = {
        "chunks_embedded": 0,
        "files_embedded": 0,
        "errors": 0,
        "skipped": 0
    }

    try:
        # Step 1: Create chunk embeddings (chunks.vector)
        logger.info(f"Creating chunk embeddings for course: {course_code}")
        embedding_create(db_path, course_code)

        # Step 2: Create file embeddings (file.vector) - MANDATORY
        if not data_dir:
            # Try to infer data_dir from course configuration
            logger.info(f"No data_dir provided, attempting to infer from course config...")
            data_dir = _infer_data_dir_from_db(db_path, course_code)
            if not data_dir:
                logger.warning(f"Could not infer data_dir for course {course_code}. File embeddings will be skipped.")
                logger.warning("This may result in incomplete embeddings in the collective database.")
                # Continue without file embeddings (degraded mode)
                stats["errors"] += 1
            else:
                logger.info(f"Inferred data_dir: {data_dir}")

        if data_dir:
            logger.info(f"Creating file embeddings for course: {course_code}")
            file_results = embed_files_from_markdown(
                db_path=db_path,
                data_dir=data_dir,
                course_filter=course_code,
                force_recompute=force_recompute
            )

            stats["files_embedded"] = file_results.get("processed", 0)
            stats["errors"] += file_results.get("errors", 0)
            stats["skipped"] = file_results.get("skipped", 0)

        # Get final embedding statistics
        embed_status = check_embedding_status(db_path, course_code)
        stats["chunks_embedded"] = embed_status.get("chunks_embedded", 0)
        stats["files_embedded"] = embed_status.get("files_embedded", 0)

        logger.info(f"Embedding creation completed. Stats: {stats}")
        return stats

    except Exception as e:
        logger.error(f"Error creating embeddings: {str(e)}")
        raise


def _infer_data_dir_from_db(db_path: str, course_code: str) -> Optional[str]:
    """
    Infer the data directory from the database configuration.

    Strategy:
    1. Look for master config to find course config path
    2. Extract output_dir from course config
    3. Validate db_path matches to ensure we're looking at the right course

    Args:
        db_path: Path to course database (used for validation)
        course_code: Course identifier

    Returns:
        Inferred data directory path or None if not found
    """
    try:
        # Try to find master config
        master_config_path = Path(__file__).parent / "configs" / "courses_master_config.yaml"
        if not master_config_path.exists():
            return None

        master_config = load_yaml(str(master_config_path))

        # Find course in master config
        for _course_name, course_info in master_config.get("courses", {}).items():
            config_path = course_info.get("config_path")
            if config_path and Path(config_path).exists():
                # Load course config
                course_config = load_yaml(str(config_path))

                # Check if this is the right course
                if course_config.get("course_code") == course_code:
                    # Validate that db_path matches (optional safety check)
                    config_db_path = course_config.get("db_path")
                    if config_db_path and str(config_db_path) != str(db_path):
                        logger.debug(f"DB path mismatch for {course_code}: {config_db_path} vs {db_path}")

                    output_dir = course_config.get("output_dir")
                    if output_dir and Path(output_dir).exists():
                        return str(output_dir)

        return None

    except Exception as e:
        logger.warning(f"Error inferring data_dir: {str(e)}")
        return None


# Removed create_embeddings_batch() - use create_embeddings_for_course() in a loop instead
# Removed process_course_pipeline() - use convert_directory() + merge separately for clarity


# ========================
# Utility Functions
# ========================

def get_processing_status(db_path: str) -> Dict[str, Any]:
    """
    Get detailed processing status for a course database.

    This is a simplified wrapper around check_embedding_status() that adds
    database existence checking and error handling.

    Args:
        db_path: Path to course database

    Returns:
        Dictionary with processing status including all embedding statistics

    Example:
        >>> status = get_processing_status("data/CS61A_metadata.db")
        >>> print(f"Files: {status['files_embedded']}/{status['total_files']}")
        >>> print(f"Chunks: {status['chunks_embedded']}/{status['total_chunks']}")
    """
    try:
        if not Path(db_path).exists():
            return {
                "error": "Database not found",
                "database_path": db_path,
                "database_exists": False
            }

        # Get comprehensive embedding status
        status = check_embedding_status(db_path)
        status["database_path"] = db_path
        status["database_exists"] = True

        return status

    except Exception as e:
        logger.error(f"Error getting processing status: {str(e)}")
        return {
            "error": str(e),
            "database_path": db_path,
            "database_exists": Path(db_path).exists()
        }


# ========================
# Database Validation Functions
# ========================

def validate_database_integrity(
    db_path: str,
    verbose: bool = True,
    save_report: Optional[str] = None
) -> Dict[str, Any]:
    """
    Validate database integrity and check for null columns.

    This function performs comprehensive validation of the database to identify:
    - Columns that are completely NULL (indicating processing errors)
    - Empty tables
    - Missing embeddings
    - Orphaned records

    Args:
        db_path: Path to the database to validate
        verbose: Whether to print detailed report to console (default: True)
        save_report: Optional path to save the validation report

    Returns:
        Dictionary containing:
            - null_columns: List of columns that are completely NULL
            - empty_tables: List of empty tables
            - integrity_issues: Count of integrity issues found
            - embedding_status: Embedding completeness statistics
            - has_critical_issues: Boolean indicating if critical issues exist
            - report: Full text report (if verbose=True)

    Example:
        >>> validation = validate_database_integrity(
        ...     "data/collective_metadata.db",
        ...     verbose=True,
        ...     save_report="validation_report.txt"
        ... )
        >>> if validation["has_critical_issues"]:
        ...     print("Critical issues found!")
        ...     for col in validation["null_columns"]:
        ...         print(f"  - {col['table']}.{col['column']} is completely NULL")
    """
    from file_conversion_router.utils.database_validator import DatabaseValidator

    try:
        validator = DatabaseValidator(db_path)

        # Run comprehensive checks
        integrity_results = validator.check_database_integrity()
        embedding_results = validator.check_embedding_completeness()

        # Generate report if requested
        report_text = ""
        if verbose or save_report:
            report_text = validator.generate_report(save_report)
            if verbose:
                print(report_text)

        # Prepare summary for return
        validation_summary = {
            "database_path": db_path,
            "null_columns": integrity_results.get("null_columns", []),
            "empty_tables": integrity_results.get("empty_tables", []),
            "warnings": integrity_results.get("warnings", []),
            "statistics": integrity_results.get("statistics", {}),
            "embedding_status": {
                "files": embedding_results.get("files", {}),
                "chunks": embedding_results.get("chunks", {}),
                "issues": embedding_results.get("issues", [])
            },
            "has_critical_issues": len(integrity_results.get("null_columns", [])) > 0,
            "integrity_issues": len(integrity_results.get("warnings", [])),
            "report": report_text if verbose else None
        }

        # Log summary
        logger.info(f"Database validation completed for: {db_path}")
        logger.info(f"  - NULL columns found: {len(validation_summary['null_columns'])}")
        logger.info(f"  - Empty tables found: {len(validation_summary['empty_tables'])}")
        logger.info(f"  - Total warnings: {validation_summary['integrity_issues']}")

        return validation_summary

    except Exception as e:
        logger.error(f"Error validating database: {str(e)}")
        raise


# ========================
# Database Merging Functions
# ========================

def merge_course_databases_with_stats(
    master_config_path: Optional[str] = None,
    exclude_test: bool = True,
    check_embeddings: bool = True
) -> Dict[str, Any]:
    """
    Merge course databases with enhanced statistics and test exclusion.

    This function merges all enabled course databases into a collective database,
    with options to exclude test databases and check embedding statistics.

    Args:
        master_config_path: Path to master configuration file
        exclude_test: Whether to exclude test/demo databases (default: True)
        check_embeddings: Whether to check embedding statistics after merge (default: True)

    Returns:
        Dictionary containing:
            - merge_stats: Database merging statistics
            - excluded_courses: List of excluded course names
            - included_courses: List of included course names
            - embedding_stats: Embedding statistics (if check_embeddings=True)
                - overall: Total files and chunks with embeddings
                - by_course: Per-course embedding statistics

    Example:
        >>> results = merge_course_databases_with_stats(
        ...     exclude_test=True,
        ...     check_embeddings=True
        ... )
        >>> print(f"Merged {len(results['included_courses'])} courses")
        >>> print(f"Files with embeddings: {results['embedding_stats']['overall']['files_embedded']}")
    """
    return merge_course_databases_from_master_config(
        master_config_path=master_config_path,
        exclude_test=exclude_test,
        check_embeddings=check_embeddings
    )


# ========================
# Batch File Conversion
# ========================

def batch_convert_files(
    files: List[Union[str, Path]],
    course_code: str,
    course_name: str,
    output_dir: Union[str, Path],
    db_path: Union[str, Path],
    input_root: Optional[Union[str, Path]] = None,
    auto_embed: bool = True,
    progress_callback: Optional[callable] = None,
) -> Dict[str, Any]:
    """
    Convert multiple files with progress tracking.

    This function processes a list of files through the conversion pipeline,
    reporting progress via an optional callback. It continues processing
    even if some files fail, collecting all errors for the final report.

    Args:
        files: List of file paths to convert
        course_code: Course identifier (e.g., "CS61A")
        course_name: Full course name
        output_dir: Directory for converted output files
        db_path: Path to SQLite database for metadata storage
        input_root: Root directory for relative path calculation.
                   If None, uses each file's parent directory.
        auto_embed: Whether to generate embeddings after conversion (default: True)
        progress_callback: Optional callback function called for each file:
                          callback(file_name: str, status: str, error: Optional[str])
                          - status: "started", "completed", "failed", "skipped"
                          - error: Error message if status is "failed"

    Returns:
        Dictionary containing:
            - files_processed: Number of files successfully converted
            - files_failed: Number of files that failed conversion
            - files_skipped: Number of files skipped (cache hits)
            - total_chunks: Total chunks created across all files
            - errors: List of error details [{file_name, error_message}]
            - results: List of per-file results [{file_name, file_uuid, chunks_count, status}]

    Example:
        >>> def on_progress(file_name, status, error):
        ...     print(f"{file_name}: {status}")
        ...
        >>> result = batch_convert_files(
        ...     files=["doc1.pdf", "doc2.html"],
        ...     course_code="CS61A",
        ...     course_name="Structure and Interpretation of Computer Programs",
        ...     output_dir="/path/to/output",
        ...     db_path="/path/to/db.sqlite",
        ...     progress_callback=on_progress
        ... )
        >>> print(f"Processed {result['files_processed']} files, {result['files_failed']} failed")
    """
    from file_conversion_router.services.directory_service import (
        process_single_file,
        ensure_chunk_db,
        converter_mapping,
    )
    from file_conversion_router.config import get_allowed_extensions

    output_dir = Path(output_dir)
    db_path = Path(db_path)
    input_root = Path(input_root) if input_root else None

    # Ensure output directory exists
    output_dir.mkdir(parents=True, exist_ok=True)

    # Initialize database connection
    conn = ensure_chunk_db(db_path)

    # Track results
    stats = {
        "files_processed": 0,
        "files_failed": 0,
        "files_skipped": 0,
        "total_chunks": 0,
        "errors": [],
        "results": [],
    }

    allowed_extensions = get_allowed_extensions()

    try:
        for file_path in files:
            file_path = Path(file_path)
            file_name = file_path.name

            # Notify progress: started
            if progress_callback:
                progress_callback(file_name, "started", None)

            # Validate file exists
            if not file_path.exists():
                error_msg = f"File not found: {file_path}"
                logger.error(error_msg)
                stats["files_failed"] += 1
                stats["errors"].append({"file_name": file_name, "error_message": error_msg})
                stats["results"].append({
                    "file_name": file_name,
                    "file_uuid": None,
                    "chunks_count": 0,
                    "status": "failed",
                    "error": error_msg,
                })
                if progress_callback:
                    progress_callback(file_name, "failed", error_msg)
                continue

            # Validate file extension
            if file_path.suffix.lower() not in allowed_extensions:
                error_msg = f"Unsupported file type: {file_path.suffix}. Allowed: {allowed_extensions}"
                logger.warning(error_msg)
                stats["files_failed"] += 1
                stats["errors"].append({"file_name": file_name, "error_message": error_msg})
                stats["results"].append({
                    "file_name": file_name,
                    "file_uuid": None,
                    "chunks_count": 0,
                    "status": "failed",
                    "error": error_msg,
                })
                if progress_callback:
                    progress_callback(file_name, "failed", error_msg)
                continue

            # Process the file
            try:
                file_input_root = input_root if input_root else file_path.parent

                chunks, metadata, file_uuid = process_single_file(
                    input_file_path=file_path,
                    output_dir=output_dir,
                    course_name=course_name,
                    course_code=course_code,
                    conn=conn,
                    input_root=file_input_root,
                )

                # Check if it was a cache hit
                if isinstance(metadata, dict) and metadata.get("cache_hit"):
                    stats["files_skipped"] += 1
                    stats["results"].append({
                        "file_name": file_name,
                        "file_uuid": file_uuid,
                        "chunks_count": 0,
                        "status": "skipped",
                    })
                    if progress_callback:
                        progress_callback(file_name, "skipped", None)
                else:
                    chunks_count = len(chunks) if chunks else 0
                    stats["files_processed"] += 1
                    stats["total_chunks"] += chunks_count
                    stats["results"].append({
                        "file_name": file_name,
                        "file_uuid": file_uuid,
                        "chunks_count": chunks_count,
                        "status": "completed",
                    })
                    if progress_callback:
                        progress_callback(file_name, "completed", None)

            except Exception as e:
                error_msg = f"{type(e).__name__}: {str(e)[:200]}"
                logger.error(f"Failed to convert {file_path}: {error_msg}")
                stats["files_failed"] += 1
                stats["errors"].append({"file_name": file_name, "error_message": error_msg})
                stats["results"].append({
                    "file_name": file_name,
                    "file_uuid": None,
                    "chunks_count": 0,
                    "status": "failed",
                    "error": error_msg,
                })
                if progress_callback:
                    progress_callback(file_name, "failed", error_msg)
                continue

        # Generate embeddings if requested and files were processed
        if auto_embed and stats["files_processed"] > 0:
            logger.info(f"Generating embeddings for {stats['files_processed']} processed files...")
            try:
                from file_conversion_router.embedding.embedding_create import embedding_create
                from file_conversion_router.embedding.file_embedding_create import embed_files_from_markdown

                # Generate chunk embeddings
                embedding_create(str(db_path), course_code)

                # Generate file embeddings
                embed_files_from_markdown(
                    db_path=str(db_path),
                    data_dir=str(output_dir),
                    course_filter=course_code,
                    force_recompute=False,
                )
                logger.info("Embedding generation completed")
            except Exception as e:
                logger.error(f"Failed to generate embeddings: {str(e)}")
                stats["embedding_error"] = str(e)

    finally:
        conn.close()

    logger.info(
        f"Batch conversion completed: {stats['files_processed']} processed, "
        f"{stats['files_failed']} failed, {stats['files_skipped']} skipped"
    )
    return stats


# Public API exports
__all__ = [
    # === Core Workflow Functions ===
    'convert_directory',                    # Step 1: Convert documents
    'process_courses_from_master_config',   # Step 2: Batch convert
    'merge_course_databases_with_stats',    # Step 3: Merge databases (recommended)
    'batch_convert_files',                  # Batch convert multiple files with progress

    # === Course Management ===
    'update_master_config_status',
    'get_courses_needing_update',
    'mark_course_for_update',

    # === Database Merging (Lower-level) ===
    'merge_course_databases_into_collective',
    'merge_all_course_databases_in_directory',
    'merge_databases_by_list',
    'merge_course_databases_from_master_config',  # Full-featured version with validation

    # === Embedding Functions ===
    'embedding_create',                     # Create chunk embeddings (chunks.vector)
    'embed_files_from_markdown',            # Create file embeddings (file.vector)
    'check_embedding_status',               # Check both chunk and file embeddings
    'create_embeddings_for_course',         # Create ALL embeddings (chunks + files)

    # === Utility Functions ===
    'get_processing_status',                # Simple status check
    'validate_database_integrity',          # Comprehensive validation

    # === YAML utilities ===
    'load_yaml',
    'save_yaml',
]


# ========================
# Example Usage
# ========================

if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # ===== RECOMMENDED WORKFLOW =====

    # Step 1: Convert and embed a single course
    convert_directory(
        "/home/bot/bot/yk/YK_final/course_yaml/CS 61A_config.yaml",
        auto_embed=True
    )

    # # Step 2: Process all courses marked for update (batch)
    # results = process_courses_from_master_config(auto_embed=True)
    # print(f"Processed courses: {results['courses_processed']}")

    # Step 3: Merge all courses into collective database
    # results = merge_course_databases_with_stats(
    #     exclude_test=True,      # Skip test/demo databases
    #     check_embeddings=True   # Validates and fixes missing embeddings before merge
    # )
    # print(f"Merged {len(results['included_courses'])} courses")
    # print(f"Embedding completeness: {results['embedding_stats']['overall']}")


    # Create embeddings for existing course (if missing)
    # stats = create_embeddings_for_course(
    #     db_path="data/CS61A_metadata.db",
    #     course_code="CS61A",
    #     data_dir=None,  # Auto-infers from config
    #     force_recompute=False
    # )
    # print(f"Embeddings: {stats['files_embedded']} files, {stats['chunks_embedded']} chunks")

    # Check processing status
    # status = get_processing_status("data/CS61A_metadata.db")
    # print(f"Files: {status['files_embedded']}/{status['total_files']}")
    # print(f"Chunks: {status['chunks_embedded']}/{status['total_chunks']}")

    # Validate database integrity
    # validation = validate_database_integrity(
    #     "data/collective_metadata.db",
    #     verbose=True,
    #     save_report="validation_report.txt"
    # )
    # if validation["has_critical_issues"]:
    #     print("Critical issues found!")

    # Mark a course for re-processing
    # mark_course_for_update("CS61A")

    # Get courses needing update
    # courses = get_courses_needing_update()
    # print(f"Courses needing update: {[c['name'] for c in courses]}")

    # logger.info("Starting batch processing with auto-embedding...")
    # results = process_courses_from_master_config(auto_embed=True)
    # logger.info(f"Batch processing completed: {results}")