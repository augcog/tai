"""Test fixtures and utilities for database testing."""

import pytest
import sqlite3
import tempfile
import uuid
from pathlib import Path
from typing import Dict, List, Tuple
import json
import shutil

from file_conversion_router.services.directory_service import SQL_INIT
from file_conversion_router.utils.database_checker import DatabaseChecker


@pytest.fixture
def temp_dir():
    """Create a temporary directory for test files."""
    temp_dir = tempfile.mkdtemp()
    yield Path(temp_dir)
    shutil.rmtree(temp_dir)


@pytest.fixture
def actual_courses():
    """Return the actual course configurations matching your system."""
    return {
        "CS 61A": {
            "course_code": "CS 61A",
            "course_name": "Structure and Interpretation of Computer Programs"
        },
        "Berkeley": {
            "course_code": "Berkeley",
            "course_name": "Berkeley Course Materials"
        },
        "CS 294-137": {
            "course_code": "CS 294-137",
            "course_name": "Large Language Models"
        },
        "ROAR Academy": {
            "course_code": "ROAR Academy",
            "course_name": "ROAR Academy Training Materials"
        }
    }


@pytest.fixture
def create_test_database():
    """Factory function to create test databases with sample data."""

    def _create_database(db_path: Path, course_config: Dict,
                        num_files: int = 1, chunks_per_file: int = 2,
                        problems_per_file: int = 1,
                        include_data_issues: bool = False) -> Dict:
        """
        Create a test database with specified parameters.

        Args:
            db_path: Path where to create the database
            course_config: Dictionary with course_code and course_name
            num_files: Number of files to create
            chunks_per_file: Number of chunks per file
            problems_per_file: Number of problems per file
            include_data_issues: Whether to include data quality issues for testing

        Returns:
            Dictionary with created data for verification
        """
        # Ensure parent directory exists
        db_path.parent.mkdir(parents=True, exist_ok=True)

        # Create database connection
        conn = sqlite3.connect(str(db_path))
        conn.executescript(SQL_INIT)
        conn.row_factory = sqlite3.Row

        course_code = course_config["course_code"]
        course_name = course_config["course_name"]

        created_data = {
            "files": [],
            "chunks": [],
            "problems": []
        }

        try:
            # Create files
            for i in range(num_files):
                file_data = {
                    "uuid": str(uuid.uuid4()),
                    "file_hash": f"hash_{course_code.replace(' ', '_')}_{i}_{uuid.uuid4().hex[:8]}",
                    "sections": json.dumps([f"Section {j+1}" for j in range(3)]),
                    "relative_path": f"lecture{i+1}.pdf",
                    "course_code": course_code,
                    "course_name": course_name,
                    "file_name": f"lecture{i+1}.pdf",
                    "extra_info": json.dumps({"page_count": 10 + i, "file_size": 1024000 + i*1000}),
                    "url": f"https://example.com/{course_code.lower().replace(' ', '')}/lecture{i+1}.pdf"
                }

                # Insert file
                conn.execute("""
                    INSERT INTO file (uuid, file_hash, sections, relative_path, course_code,
                                    course_name, file_name, extra_info, url)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    file_data["uuid"], file_data["file_hash"], file_data["sections"],
                    file_data["relative_path"], file_data["course_code"], file_data["course_name"],
                    file_data["file_name"], file_data["extra_info"], file_data["url"]
                ))

                created_data["files"].append(file_data)

                # Create chunks for this file
                for j in range(chunks_per_file):
                    chunk_data = {
                        "chunk_uuid": str(uuid.uuid4()),
                        "file_uuid": file_data["uuid"],
                        "idx": j,
                        "text": f"This is chunk {j+1} content for {course_code} lecture {i+1}." if not (include_data_issues and j == 0) else "",  # Empty text for first chunk if testing issues
                        "title": f"Section {j+1}",
                        "url": f"{file_data['url']}#page={j+1}",
                        "file_path": f"/path/to/{file_data['file_name']}",
                        "reference_path": f"{course_code.lower().replace(' ', '_')}/lectures/{file_data['file_name']}",
                        "course_name": course_name,
                        "course_code": course_code,
                        "chunk_index": j
                    }

                    # Insert chunk
                    conn.execute("""
                        INSERT INTO chunks (chunk_uuid, file_uuid, idx, text, title, url,
                                          file_path, reference_path, course_name, course_code, chunk_index)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """, (
                        chunk_data["chunk_uuid"], chunk_data["file_uuid"], chunk_data["idx"],
                        chunk_data["text"], chunk_data["title"], chunk_data["url"],
                        chunk_data["file_path"], chunk_data["reference_path"], chunk_data["course_name"],
                        chunk_data["course_code"], chunk_data["chunk_index"]
                    ))

                    created_data["chunks"].append(chunk_data)

                # Create problems for this file
                for k in range(problems_per_file):
                    problem_data = {
                        "uuid": str(uuid.uuid4()),
                        "file_uuid": file_data["uuid"],
                        "problem_index": k + 1,
                        "problem_id": f"prob_{i+1}_{k+1}",
                        "problem_content": f"Problem {k+1} content for {course_code} lecture {i+1}.",
                        "question_id": f"q_{i+1}_{k+1}",
                        "question": f"What is the answer to problem {k+1}?",
                        "choices": json.dumps([f"Option {x}" for x in ["A", "B", "C", "D"]]),
                        "answer": "Option A",
                        "explanation": f"This is the explanation for problem {k+1}.",
                        "question_type": "multiple_choice"
                    }

                    # Insert problem
                    conn.execute("""
                        INSERT INTO problem (uuid, file_uuid, problem_index, problem_id, problem_content,
                                           question_id, question, choices, answer, explanation, question_type)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """, (
                        problem_data["uuid"], problem_data["file_uuid"], problem_data["problem_index"],
                        problem_data["problem_id"], problem_data["problem_content"], problem_data["question_id"],
                        problem_data["question"], problem_data["choices"], problem_data["answer"],
                        problem_data["explanation"], problem_data["question_type"]
                    ))

                    created_data["problems"].append(problem_data)

            # Add orphaned chunks if testing data issues
            if include_data_issues:
                orphaned_chunk = {
                    "chunk_uuid": str(uuid.uuid4()),
                    "file_uuid": str(uuid.uuid4()),  # Non-existent file UUID
                    "idx": 99,
                    "text": "This is an orphaned chunk",
                    "title": "Orphaned Section",
                    "url": "https://example.com/orphaned",
                    "file_path": "/path/to/orphaned.pdf",
                    "reference_path": "orphaned/orphaned.pdf",
                    "course_name": course_name,
                    "course_code": course_code,
                    "chunk_index": 99
                }

                conn.execute("""
                    INSERT INTO chunks (chunk_uuid, file_uuid, idx, text, title, url,
                                      file_path, reference_path, course_name, course_code, chunk_index)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    orphaned_chunk["chunk_uuid"], orphaned_chunk["file_uuid"], orphaned_chunk["idx"],
                    orphaned_chunk["text"], orphaned_chunk["title"], orphaned_chunk["url"],
                    orphaned_chunk["file_path"], orphaned_chunk["reference_path"], orphaned_chunk["course_name"],
                    orphaned_chunk["course_code"], orphaned_chunk["chunk_index"]
                ))

                created_data["chunks"].append(orphaned_chunk)

            conn.commit()

        finally:
            conn.close()

        return created_data

    return _create_database


@pytest.fixture
def sample_course_databases(temp_dir, create_test_database, actual_courses):
    """Create sample course databases matching your actual courses."""
    databases = {}

    for course_key, course_config in actual_courses.items():
        db_path = temp_dir / f"{course_key.replace(' ', '_')}_metadata.db"

        # Vary the data for each course
        num_files = {"CS 61A": 2, "Berkeley": 3, "CS 294-137": 1, "ROAR Academy": 2}[course_key]
        chunks_per_file = {"CS 61A": 3, "Berkeley": 2, "CS 294-137": 4, "ROAR Academy": 2}[course_key]
        problems_per_file = {"CS 61A": 2, "Berkeley": 1, "CS 294-137": 3, "ROAR Academy": 1}[course_key]

        course_data = create_test_database(
            db_path, course_config,
            num_files=num_files,
            chunks_per_file=chunks_per_file,
            problems_per_file=problems_per_file
        )

        databases[course_key] = {"path": db_path, "data": course_data, "config": course_config}

    return databases


@pytest.fixture
def sample_database_with_issues(temp_dir, create_test_database, actual_courses):
    """Create a database with known data quality issues for testing validation."""
    course_config = actual_courses["CS 61A"]
    db_path = temp_dir / "test_issues_metadata.db"

    data = create_test_database(
        db_path, course_config,
        num_files=2, chunks_per_file=2, problems_per_file=1,
        include_data_issues=True
    )

    return {"path": db_path, "data": data, "config": course_config}


@pytest.fixture
def empty_collective_db_path(temp_dir):
    """Path for an empty collective database."""
    return temp_dir / "collective_metadata.db"


def get_database_stats(db_path: Path) -> Dict[str, int]:
    """Get statistics from a database."""
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row

    try:
        stats = {}

        # Count files
        stats["files"] = conn.execute("SELECT COUNT(*) FROM file").fetchone()[0]

        # Count chunks
        stats["chunks"] = conn.execute("SELECT COUNT(*) FROM chunks").fetchone()[0]

        # Count problems
        stats["problems"] = conn.execute("SELECT COUNT(*) FROM problem").fetchone()[0]

        # Get course codes
        course_codes = conn.execute(
            "SELECT DISTINCT course_code FROM file WHERE course_code IS NOT NULL"
        ).fetchall()
        stats["course_codes"] = [row[0] for row in course_codes]

        return stats

    finally:
        conn.close()


def verify_database_integrity(db_path: Path) -> List[str]:
    """Verify database integrity using the DatabaseChecker."""
    checker = DatabaseChecker(str(db_path))
    report = checker.check_all()

    # Extract all issues from the report
    all_issues = []
    for section, data in report.items():
        if isinstance(data, dict) and "issues" in data:
            all_issues.extend(data["issues"])

    return all_issues


@pytest.fixture
def database_checker():
    """Provide DatabaseChecker class for testing."""
    return DatabaseChecker