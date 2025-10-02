"""Database integrity and data quality checker utility."""

import sqlite3
import json
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
import logging


class DatabaseChecker:
    """Comprehensive database integrity and data quality checker."""

    def __init__(self, db_path: str):
        """Initialize with database path."""
        self.db_path = Path(db_path)
        if not self.db_path.exists():
            raise FileNotFoundError(f"Database not found: {db_path}")

    def check_all(self) -> Dict[str, Any]:
        """Run all checks and return comprehensive report."""
        report = {
            "database_path": str(self.db_path),
            "schema_check": self.check_schema(),
            "data_types": self.check_data_types(),
            "missing_data": self.check_missing_data(),
            "referential_integrity": self.check_referential_integrity(),
            "data_consistency": self.check_data_consistency(),
            "statistics": self.get_statistics()
        }

        # Count total issues
        total_issues = 0
        for check_name, check_result in report.items():
            if isinstance(check_result, dict) and "issues" in check_result:
                total_issues += len(check_result["issues"])

        report["summary"] = {
            "total_issues": total_issues,
            "status": "PASS" if total_issues == 0 else "FAIL"
        }

        return report

    def check_schema(self) -> Dict[str, Any]:
        """Check if database has expected schema."""
        conn = sqlite3.connect(str(self.db_path))

        try:
            cursor = conn.cursor()

            # Get all tables
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
            tables = [row[0] for row in cursor.fetchall()]

            expected_tables = ["file", "chunks", "problem"]
            missing_tables = [table for table in expected_tables if table not in tables]
            extra_tables = [table for table in tables if table not in expected_tables]

            # Check table schemas
            table_schemas = {}
            for table in tables:
                cursor.execute(f"PRAGMA table_info({table})")
                columns = cursor.fetchall()
                table_schemas[table] = {
                    "columns": [{"name": col[1], "type": col[2], "not_null": bool(col[3]),
                               "default": col[4], "pk": bool(col[5])} for col in columns]
                }

            return {
                "tables_found": tables,
                "missing_tables": missing_tables,
                "extra_tables": extra_tables,
                "table_schemas": table_schemas,
                "issues": missing_tables + [f"Extra table: {t}" for t in extra_tables]
            }

        finally:
            conn.close()

    def check_data_types(self) -> Dict[str, Any]:
        """Check data types in each column."""
        conn = sqlite3.connect(str(self.db_path))

        try:
            issues = []
            type_analysis = {}

            # Check file table
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM file LIMIT 100")  # Sample for analysis
            files = cursor.fetchall()

            if files:
                # Get column names
                cursor.execute("PRAGMA table_info(file)")
                file_columns = [col[1] for col in cursor.fetchall()]

                type_analysis["file"] = self._analyze_column_types(files, file_columns)

                # Check specific validations
                for i, row in enumerate(files):
                    row_dict = dict(zip(file_columns, row))

                    # Check UUID format
                    if row_dict.get("uuid") and len(row_dict["uuid"]) != 36:
                        issues.append(f"File row {i}: Invalid UUID format")

                    # Check if sections is valid JSON when present
                    if row_dict.get("sections"):
                        try:
                            json.loads(row_dict["sections"])
                        except (json.JSONDecodeError, TypeError):
                            issues.append(f"File row {i}: Invalid JSON in sections field")

                    # Check if extra_info is valid JSON when present
                    if row_dict.get("extra_info"):
                        try:
                            json.loads(row_dict["extra_info"])
                        except (json.JSONDecodeError, TypeError):
                            issues.append(f"File row {i}: Invalid JSON in extra_info field")

            # Check chunks table
            cursor.execute("SELECT * FROM chunks LIMIT 100")
            chunks = cursor.fetchall()

            if chunks:
                cursor.execute("PRAGMA table_info(chunks)")
                chunk_columns = [col[1] for col in cursor.fetchall()]

                type_analysis["chunks"] = self._analyze_column_types(chunks, chunk_columns)

                for i, row in enumerate(chunks):
                    row_dict = dict(zip(chunk_columns, row))

                    # Check UUID formats
                    if row_dict.get("chunk_uuid") and len(row_dict["chunk_uuid"]) != 36:
                        issues.append(f"Chunk row {i}: Invalid chunk_uuid format")
                    if row_dict.get("file_uuid") and len(row_dict["file_uuid"]) != 36:
                        issues.append(f"Chunk row {i}: Invalid file_uuid format")

                    # Check numeric fields
                    if row_dict.get("idx") is not None and not isinstance(row_dict["idx"], int):
                        issues.append(f"Chunk row {i}: idx should be integer")
                    if row_dict.get("chunk_index") is not None and not isinstance(row_dict["chunk_index"], int):
                        issues.append(f"Chunk row {i}: chunk_index should be integer")

            # Check problem table
            cursor.execute("SELECT * FROM problem LIMIT 100")
            problems = cursor.fetchall()

            if problems:
                cursor.execute("PRAGMA table_info(problem)")
                problem_columns = [col[1] for col in cursor.fetchall()]

                type_analysis["problem"] = self._analyze_column_types(problems, problem_columns)

                for i, row in enumerate(problems):
                    row_dict = dict(zip(problem_columns, row))

                    # Check UUID formats
                    if row_dict.get("uuid") and len(row_dict["uuid"]) != 36:
                        issues.append(f"Problem row {i}: Invalid uuid format")
                    if row_dict.get("file_uuid") and len(row_dict["file_uuid"]) != 36:
                        issues.append(f"Problem row {i}: Invalid file_uuid format")

                    # Check if choices is valid JSON when present
                    if row_dict.get("choices"):
                        try:
                            json.loads(row_dict["choices"])
                        except (json.JSONDecodeError, TypeError):
                            issues.append(f"Problem row {i}: Invalid JSON in choices field")

            return {
                "type_analysis": type_analysis,
                "issues": issues
            }

        finally:
            conn.close()

    def _analyze_column_types(self, rows: List[Tuple], columns: List[str]) -> Dict[str, Any]:
        """Analyze data types in columns."""
        analysis = {}

        for col_idx, col_name in enumerate(columns):
            values = [row[col_idx] for row in rows if row[col_idx] is not None]

            if not values:
                analysis[col_name] = {"type": "NULL", "null_count": len(rows)}
                continue

            types = set(type(v).__name__ for v in values)
            analysis[col_name] = {
                "types_found": list(types),
                "null_count": len(rows) - len(values),
                "sample_values": values[:5] if values else []
            }

        return analysis

    def check_missing_data(self) -> Dict[str, Any]:
        """Check for missing required data."""
        conn = sqlite3.connect(str(self.db_path))

        try:
            issues = []
            missing_stats = {}

            # Check file table required fields
            cursor = conn.cursor()

            # Files with missing critical fields
            cursor.execute("SELECT COUNT(*) FROM file WHERE uuid IS NULL OR uuid = ''")
            missing_file_uuids = cursor.fetchone()[0]
            if missing_file_uuids > 0:
                issues.append(f"{missing_file_uuids} files with missing UUID")

            cursor.execute("SELECT COUNT(*) FROM file WHERE file_hash IS NULL OR file_hash = ''")
            missing_file_hashes = cursor.fetchone()[0]
            if missing_file_hashes > 0:
                issues.append(f"{missing_file_hashes} files with missing file_hash")

            cursor.execute("SELECT COUNT(*) FROM file WHERE course_code IS NULL OR course_code = ''")
            missing_course_codes = cursor.fetchone()[0]
            if missing_course_codes > 0:
                issues.append(f"{missing_course_codes} files with missing course_code")

            cursor.execute("SELECT COUNT(*) FROM file WHERE course_name IS NULL OR course_name = ''")
            missing_course_names = cursor.fetchone()[0]
            if missing_course_names > 0:
                issues.append(f"{missing_course_names} files with missing course_name")

            # Check chunks table required fields
            cursor.execute("SELECT COUNT(*) FROM chunks WHERE chunk_uuid IS NULL OR chunk_uuid = ''")
            missing_chunk_uuids = cursor.fetchone()[0]
            if missing_chunk_uuids > 0:
                issues.append(f"{missing_chunk_uuids} chunks with missing chunk_uuid")

            cursor.execute("SELECT COUNT(*) FROM chunks WHERE file_uuid IS NULL OR file_uuid = ''")
            missing_chunk_file_uuids = cursor.fetchone()[0]
            if missing_chunk_file_uuids > 0:
                issues.append(f"{missing_chunk_file_uuids} chunks with missing file_uuid")

            cursor.execute("SELECT COUNT(*) FROM chunks WHERE text IS NULL OR text = ''")
            missing_chunk_text = cursor.fetchone()[0]
            if missing_chunk_text > 0:
                issues.append(f"{missing_chunk_text} chunks with missing text")

            # Check problem table required fields
            cursor.execute("SELECT COUNT(*) FROM problem WHERE uuid IS NULL OR uuid = ''")
            missing_problem_uuids = cursor.fetchone()[0]
            if missing_problem_uuids > 0:
                issues.append(f"{missing_problem_uuids} problems with missing UUID")

            cursor.execute("SELECT COUNT(*) FROM problem WHERE file_uuid IS NULL OR file_uuid = ''")
            missing_problem_file_uuids = cursor.fetchone()[0]
            if missing_problem_file_uuids > 0:
                issues.append(f"{missing_problem_file_uuids} problems with missing file_uuid")

            missing_stats = {
                "files": {
                    "missing_uuid": missing_file_uuids,
                    "missing_file_hash": missing_file_hashes,
                    "missing_course_code": missing_course_codes,
                    "missing_course_name": missing_course_names
                },
                "chunks": {
                    "missing_chunk_uuid": missing_chunk_uuids,
                    "missing_file_uuid": missing_chunk_file_uuids,
                    "missing_text": missing_chunk_text
                },
                "problems": {
                    "missing_uuid": missing_problem_uuids,
                    "missing_file_uuid": missing_problem_file_uuids
                }
            }

            return {
                "missing_stats": missing_stats,
                "issues": issues
            }

        finally:
            conn.close()

    def check_referential_integrity(self) -> Dict[str, Any]:
        """Check referential integrity constraints."""
        conn = sqlite3.connect(str(self.db_path))

        try:
            issues = []

            cursor = conn.cursor()

            # Check chunks referencing non-existent files
            cursor.execute("""
                SELECT COUNT(*) FROM chunks
                WHERE file_uuid NOT IN (SELECT uuid FROM file WHERE uuid IS NOT NULL)
            """)
            orphaned_chunks = cursor.fetchone()[0]
            if orphaned_chunks > 0:
                issues.append(f"{orphaned_chunks} chunks reference non-existent files")

            # Check problems referencing non-existent files
            cursor.execute("""
                SELECT COUNT(*) FROM problem
                WHERE file_uuid NOT IN (SELECT uuid FROM file WHERE uuid IS NOT NULL)
            """)
            orphaned_problems = cursor.fetchone()[0]
            if orphaned_problems > 0:
                issues.append(f"{orphaned_problems} problems reference non-existent files")

            # Check for duplicate UUIDs
            cursor.execute("""
                SELECT uuid, COUNT(*) as count FROM file
                WHERE uuid IS NOT NULL
                GROUP BY uuid HAVING COUNT(*) > 1
            """)
            duplicate_file_uuids = cursor.fetchall()
            if duplicate_file_uuids:
                issues.append(f"{len(duplicate_file_uuids)} duplicate file UUIDs found")

            cursor.execute("""
                SELECT chunk_uuid, COUNT(*) as count FROM chunks
                WHERE chunk_uuid IS NOT NULL
                GROUP BY chunk_uuid HAVING COUNT(*) > 1
            """)
            duplicate_chunk_uuids = cursor.fetchall()
            if duplicate_chunk_uuids:
                issues.append(f"{len(duplicate_chunk_uuids)} duplicate chunk UUIDs found")

            cursor.execute("""
                SELECT uuid, COUNT(*) as count FROM problem
                WHERE uuid IS NOT NULL
                GROUP BY uuid HAVING COUNT(*) > 1
            """)
            duplicate_problem_uuids = cursor.fetchall()
            if duplicate_problem_uuids:
                issues.append(f"{len(duplicate_problem_uuids)} duplicate problem UUIDs found")

            return {
                "orphaned_chunks": orphaned_chunks,
                "orphaned_problems": orphaned_problems,
                "duplicate_file_uuids": len(duplicate_file_uuids),
                "duplicate_chunk_uuids": len(duplicate_chunk_uuids),
                "duplicate_problem_uuids": len(duplicate_problem_uuids),
                "issues": issues
            }

        finally:
            conn.close()

    def check_data_consistency(self) -> Dict[str, Any]:
        """Check data consistency across tables."""
        conn = sqlite3.connect(str(self.db_path))

        try:
            issues = []

            cursor = conn.cursor()

            # Check course code consistency between file and chunks
            cursor.execute("""
                SELECT DISTINCT f.course_code as file_course, c.course_code as chunk_course
                FROM file f
                JOIN chunks c ON f.uuid = c.file_uuid
                WHERE f.course_code != c.course_code OR
                      (f.course_code IS NULL AND c.course_code IS NOT NULL) OR
                      (f.course_code IS NOT NULL AND c.course_code IS NULL)
            """)
            course_code_mismatches = cursor.fetchall()
            if course_code_mismatches:
                issues.append(f"{len(course_code_mismatches)} course code mismatches between files and chunks")

            # Check course name consistency
            cursor.execute("""
                SELECT DISTINCT f.course_name as file_name, c.course_name as chunk_name
                FROM file f
                JOIN chunks c ON f.uuid = c.file_uuid
                WHERE f.course_name != c.course_name OR
                      (f.course_name IS NULL AND c.course_name IS NOT NULL) OR
                      (f.course_name IS NOT NULL AND c.course_name IS NULL)
            """)
            course_name_mismatches = cursor.fetchall()
            if course_name_mismatches:
                issues.append(f"{len(course_name_mismatches)} course name mismatches between files and chunks")

            # Check for files without chunks
            cursor.execute("""
                SELECT COUNT(*) FROM file f
                LEFT JOIN chunks c ON f.uuid = c.file_uuid
                WHERE c.file_uuid IS NULL
            """)
            files_without_chunks = cursor.fetchone()[0]
            if files_without_chunks > 0:
                issues.append(f"{files_without_chunks} files have no associated chunks")

            # Check for duplicate file hashes
            cursor.execute("""
                SELECT file_hash, COUNT(*) as count FROM file
                WHERE file_hash IS NOT NULL
                GROUP BY file_hash HAVING COUNT(*) > 1
            """)
            duplicate_hashes = cursor.fetchall()
            if duplicate_hashes:
                issues.append(f"{len(duplicate_hashes)} duplicate file hashes found")

            return {
                "course_code_mismatches": len(course_code_mismatches),
                "course_name_mismatches": len(course_name_mismatches),
                "files_without_chunks": files_without_chunks,
                "duplicate_hashes": len(duplicate_hashes),
                "issues": issues
            }

        finally:
            conn.close()

    def get_statistics(self) -> Dict[str, Any]:
        """Get database statistics."""
        conn = sqlite3.connect(str(self.db_path))

        try:
            cursor = conn.cursor()

            # Basic counts
            cursor.execute("SELECT COUNT(*) FROM file")
            file_count = cursor.fetchone()[0]

            cursor.execute("SELECT COUNT(*) FROM chunks")
            chunk_count = cursor.fetchone()[0]

            cursor.execute("SELECT COUNT(*) FROM problem")
            problem_count = cursor.fetchone()[0]

            # Course statistics
            cursor.execute("SELECT DISTINCT course_code FROM file WHERE course_code IS NOT NULL")
            courses = [row[0] for row in cursor.fetchall()]

            course_stats = {}
            for course in courses:
                cursor.execute("SELECT COUNT(*) FROM file WHERE course_code = ?", (course,))
                course_files = cursor.fetchone()[0]

                cursor.execute("SELECT COUNT(*) FROM chunks WHERE course_code = ?", (course,))
                course_chunks = cursor.fetchone()[0]

                cursor.execute("""
                    SELECT COUNT(*) FROM problem p
                    JOIN file f ON p.file_uuid = f.uuid
                    WHERE f.course_code = ?
                """, (course,))
                course_problems = cursor.fetchone()[0]

                course_stats[course] = {
                    "files": course_files,
                    "chunks": course_chunks,
                    "problems": course_problems
                }

            return {
                "total_files": file_count,
                "total_chunks": chunk_count,
                "total_problems": problem_count,
                "courses": courses,
                "course_statistics": course_stats
            }

        finally:
            conn.close()


def check_database(db_path: str, verbose: bool = True) -> Dict[str, Any]:
    """Convenient function to check a database and optionally print results."""
    checker = DatabaseChecker(db_path)
    report = checker.check_all()

    if verbose:
        print(f"\n=== Database Check Report: {db_path} ===")
        print(f"Status: {report['summary']['status']}")
        print(f"Total Issues: {report['summary']['total_issues']}")

        if report['summary']['total_issues'] > 0:
            print("\n--- Issues Found ---")
            for section, data in report.items():
                if isinstance(data, dict) and "issues" in data and data["issues"]:
                    print(f"\n{section.upper()}:")
                    for issue in data["issues"]:
                        print(f"  - {issue}")

        print(f"\n--- Statistics ---")
        stats = report['statistics']
        print(f"Files: {stats['total_files']}")
        print(f"Chunks: {stats['total_chunks']}")
        print(f"Problems: {stats['total_problems']}")
        print(f"Courses: {', '.join(stats['courses'])}")

    return report


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python database_checker.py <db_path>")
        sys.exit(1)

    db_path = sys.argv[1]
    check_database(db_path, verbose=True)