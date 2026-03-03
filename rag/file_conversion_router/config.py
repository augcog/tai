#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Centralized path configuration for file_conversion_router.

This module provides a single source of truth for all directory paths used throughout
the project. Paths are computed relative to the project root and can be overridden
via environment variables for flexibility across different environments.

Environment Variables:
    TAI_BASE_DIR: Override the base directory (default: parent of rag directory)
    TAI_COURSES_DIR: Override courses directory
    TAI_COURSES_OUT_DIR: Override courses output directory
    TAI_CONFIG_DIR: Override configuration directory
    TAI_TEST_DIR: Override test data directory
"""

import os
from pathlib import Path
from typing import Optional


# Determine project root
# file_conversion_router/config.py -> rag/file_conversion_router/config.py
# We want to go up to the 'tai' directory (3 levels up from this file)
_THIS_FILE = Path(__file__).resolve()
_FILE_CONVERSION_ROUTER_DIR = _THIS_FILE.parent  # file_conversion_router/
_RAG_DIR = _FILE_CONVERSION_ROUTER_DIR.parent    # rag/
_TAI_DIR = _RAG_DIR.parent                       # tai/

# Base directory (can be overridden via environment variable)
BASE_DIR = Path(os.getenv('TAI_BASE_DIR', str(_TAI_DIR)))

# Project structure directories
RAG_DIR = _RAG_DIR
FILE_CONVERSION_ROUTER_DIR = _FILE_CONVERSION_ROUTER_DIR

# Data directories (can be overridden via environment variables)
COURSES_DIR = Path(os.getenv('TAI_COURSES_DIR', str(BASE_DIR / 'courses')))
COURSES_OUT_DIR = Path(os.getenv('TAI_COURSES_OUT_DIR', str(BASE_DIR / 'courses_out')))
TEST_FOLDER_DIR = Path(os.getenv('TAI_TEST_DIR', str(BASE_DIR / 'test_folder')))
TEST_FOLDER_OUTPUT_DIR = Path(os.getenv('TAI_TEST_OUTPUT_DIR', str(BASE_DIR / 'test_folder_output')))

# Configuration directories
CONFIG_DIR = Path(os.getenv('TAI_CONFIG_DIR', str(BASE_DIR / 'course_yaml')))
MASTER_CONFIG_PATH = FILE_CONVERSION_ROUTER_DIR / 'configs' / 'courses_master_config.yaml'

# Database paths
COLLECTIVE_DB_PATH = COURSES_OUT_DIR / 'collective_metadata.db'
DB_DIR = COURSES_OUT_DIR / 'db'

# Batch Upload Configuration
BATCH_UPLOAD_CONFIG = {
    "max_file_size_mb": 100,           # Max size per file in MB
    "max_total_size_mb": 500,          # Max total upload size in MB
    "max_files_per_batch": 100,        # Max files per batch upload
    "allowed_extensions": [
        ".pdf", ".md", ".html", ".ipynb", ".py", ".rst",
        ".mp4", ".mkv", ".webm", ".mov"
    ],
    "temp_dir": Path(os.getenv('RAG_TEMP_DIR', '/tmp/rag_batch_uploads')),
    "cleanup_delay_seconds": 3600,     # Cleanup temp files after 1 hour
}

# Helper functions for batch upload config
def get_allowed_extensions() -> list:
    """Get list of allowed file extensions for batch upload."""
    return BATCH_UPLOAD_CONFIG["allowed_extensions"]

def get_max_file_size_bytes() -> int:
    """Get max file size in bytes."""
    return BATCH_UPLOAD_CONFIG["max_file_size_mb"] * 1024 * 1024

def get_max_total_size_bytes() -> int:
    """Get max total upload size in bytes."""
    return BATCH_UPLOAD_CONFIG["max_total_size_mb"] * 1024 * 1024

def get_temp_upload_dir() -> Path:
    """Get temp directory for batch uploads, creating if needed."""
    temp_dir = BATCH_UPLOAD_CONFIG["temp_dir"]
    temp_dir.mkdir(parents=True, exist_ok=True)
    return temp_dir


def get_course_db_path(course_code: str) -> Path:
    """Get the database path for a specific course.

    Args:
        course_code: Course identifier (e.g., "CS61A")

    Returns:
        Path to the course's metadata database

    Example:
        >>> db_path = get_course_db_path("CS61A")
        >>> print(db_path)
        /path/to/courses_out/db/CS61A_metadata.db
    """
    return DB_DIR / f"{course_code}_metadata.db"


def get_course_output_dir(course_code: str) -> Path:
    """Get the output directory for a specific course.

    Args:
        course_code: Course identifier (e.g., "CS61A")

    Returns:
        Path to the course's output directory

    Example:
        >>> output_dir = get_course_output_dir("CS61A")
        >>> print(output_dir)
        /path/to/courses_out/CS61A
    """
    return COURSES_OUT_DIR / course_code


def get_test_data_path(relative_path: str) -> Path:
    """Get a path within the test data directory.

    Args:
        relative_path: Path relative to test folder

    Returns:
        Absolute path to the test data

    Example:
        >>> test_pdf = get_test_data_path("testing/pdfs/disc01.pdf")
        >>> print(test_pdf)
        /path/to/test_folder/testing/pdfs/disc01.pdf
    """
    return TEST_FOLDER_DIR / relative_path


def get_test_output_path(relative_path: str) -> Path:
    """Get a path within the test output directory.

    Args:
        relative_path: Path relative to test output folder

    Returns:
        Absolute path to the test output location

    Example:
        >>> output = get_test_output_path("disc01/result.md")
        >>> print(output)
        /path/to/test_folder_output/disc01/result.md
    """
    return TEST_FOLDER_OUTPUT_DIR / relative_path


def ensure_directories_exist(*dirs: Path) -> None:
    """Create directories if they don't exist.

    Args:
        *dirs: Variable number of Path objects to create

    Example:
        >>> ensure_directories_exist(COURSES_OUT_DIR, DB_DIR)
    """
    for directory in dirs:
        directory.mkdir(parents=True, exist_ok=True)


def get_relative_path(absolute_path: Path, base: Optional[Path] = None) -> Path:
    """Convert an absolute path to a relative path from a base directory.

    Args:
        absolute_path: The absolute path to convert
        base: Base directory (defaults to BASE_DIR)

    Returns:
        Relative path from base to absolute_path

    Example:
        >>> abs_path = Path("/path/to/tai/courses/CS61A/file.pdf")
        >>> rel_path = get_relative_path(abs_path)
        >>> print(rel_path)
        courses/CS61A/file.pdf
    """
    if base is None:
        base = BASE_DIR

    try:
        return absolute_path.relative_to(base)
    except ValueError:
        # If paths are not relative, return the original path
        return absolute_path


# Print configuration on import (useful for debugging)
def _print_config():
    """Print current path configuration (for debugging)."""
    print("=" * 60)
    print("File Conversion Router - Path Configuration")
    print("=" * 60)
    print(f"BASE_DIR:                {BASE_DIR}")
    print(f"RAG_DIR:                 {RAG_DIR}")
    print(f"FILE_CONV_ROUTER_DIR:    {FILE_CONVERSION_ROUTER_DIR}")
    print(f"COURSES_DIR:             {COURSES_DIR}")
    print(f"COURSES_OUT_DIR:         {COURSES_OUT_DIR}")
    print(f"TEST_FOLDER_DIR:         {TEST_FOLDER_DIR}")
    print(f"TEST_FOLDER_OUTPUT_DIR:  {TEST_FOLDER_OUTPUT_DIR}")
    print(f"CONFIG_DIR:              {CONFIG_DIR}")
    print(f"MASTER_CONFIG_PATH:      {MASTER_CONFIG_PATH}")
    print(f"COLLECTIVE_DB_PATH:      {COLLECTIVE_DB_PATH}")
    print(f"DB_DIR:                  {DB_DIR}")
    print("=" * 60)


# Only print config if run as main module
if __name__ == "__main__":
    _print_config()

    # Validate that key directories exist or can be created
    print("\nDirectory Status:")
    for name, path in [
        ("BASE_DIR", BASE_DIR),
        ("RAG_DIR", RAG_DIR),
        ("FILE_CONVERSION_ROUTER_DIR", FILE_CONVERSION_ROUTER_DIR),
        ("COURSES_DIR", COURSES_DIR),
        ("COURSES_OUT_DIR", COURSES_OUT_DIR),
    ]:
        exists = "✓ EXISTS" if path.exists() else "✗ MISSING"
        print(f"  {name:30s} {exists}")
