"""Database integrity validation utilities for checking column nullness and data quality."""

import sqlite3
import logging
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from datetime import datetime

logger = logging.getLogger(__name__)


class DatabaseValidator:
    """Validates database integrity and identifies data quality issues."""

    def __init__(self, db_path: str):
        """
        Initialize database validator.

        Args:
            db_path: Path to the SQLite database
        """
        self.db_path = db_path
        if not Path(db_path).exists():
            raise FileNotFoundError(f"Database not found: {db_path}")

    def check_database_integrity(self) -> Dict[str, Any]:
        """
        Perform comprehensive database integrity check.

        Returns:
            Dictionary containing:
                - tables: List of tables and their column status
                - null_columns: Columns that are completely null
                - empty_tables: Tables with no data
                - warnings: List of potential issues
                - statistics: Overall database statistics
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        try:
            results = {
                "database_path": self.db_path,
                "check_timestamp": datetime.now().isoformat(),
                "tables": {},
                "null_columns": [],
                "empty_tables": [],
                "warnings": [],
                "statistics": {}
            }

            # Get all tables
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
            tables = cursor.fetchall()

            total_tables = len(tables)
            total_columns_checked = 0
            total_null_columns = 0

            for table_name in tables:
                table_name = table_name[0]

                # Skip SQLite internal tables
                if table_name.startswith('sqlite_'):
                    continue

                table_info = self._analyze_table(conn, table_name)
                results["tables"][table_name] = table_info

                # Track empty tables
                if table_info["row_count"] == 0:
                    results["empty_tables"].append(table_name)
                    results["warnings"].append(f"Table '{table_name}' is empty")

                # Track completely null columns
                for column_name, column_data in table_info["columns"].items():
                    total_columns_checked += 1

                    if column_data["all_null"]:
                        total_null_columns += 1
                        null_info = {
                            "table": table_name,
                            "column": column_name,
                            "row_count": table_info["row_count"]
                        }
                        results["null_columns"].append(null_info)
                        results["warnings"].append(
                            f"Column '{table_name}.{column_name}' is completely NULL"
                        )

                    # Check for high null percentage (but not 100%)
                    elif column_data["null_percentage"] > 90 and column_data["null_percentage"] < 100:
                        results["warnings"].append(
                            f"Column '{table_name}.{column_name}' has {column_data['null_percentage']:.1f}% NULL values"
                        )

            # Add overall statistics
            results["statistics"] = {
                "total_tables": total_tables,
                "empty_tables": len(results["empty_tables"]),
                "total_columns_checked": total_columns_checked,
                "completely_null_columns": total_null_columns,
                "tables_with_issues": len(results["empty_tables"]) + len(set(col["table"] for col in results["null_columns"]))
            }

            # Check specific critical columns
            self._check_critical_columns(conn, results)

            return results

        finally:
            conn.close()

    def _analyze_table(self, conn: sqlite3.Connection, table_name: str) -> Dict[str, Any]:
        """
        Analyze a single table for data quality issues.

        Args:
            conn: Database connection
            table_name: Name of the table to analyze

        Returns:
            Dictionary with table analysis results
        """
        cursor = conn.cursor()

        # Get table schema
        cursor.execute(f"PRAGMA table_info({table_name})")
        columns = cursor.fetchall()

        # Get row count
        cursor.execute(f"SELECT COUNT(*) FROM {table_name}")
        row_count = cursor.fetchone()[0]

        table_info = {
            "row_count": row_count,
            "column_count": len(columns),
            "columns": {}
        }

        # Analyze each column
        for col in columns:
            col_name = col[1]
            col_type = col[2]
            is_nullable = col[3] == 0  # notnull flag is 0 for nullable columns

            # Count NULL values
            cursor.execute(f"SELECT COUNT(*) FROM {table_name} WHERE {col_name} IS NULL")
            null_count = cursor.fetchone()[0]

            # Count non-NULL values
            non_null_count = row_count - null_count

            # Check if column is completely NULL
            all_null = (null_count == row_count and row_count > 0)

            # Calculate null percentage
            null_percentage = (null_count / row_count * 100) if row_count > 0 else 0

            table_info["columns"][col_name] = {
                "type": col_type,
                "nullable": is_nullable,
                "null_count": null_count,
                "non_null_count": non_null_count,
                "all_null": all_null,
                "null_percentage": null_percentage
            }

        return table_info

    def _check_critical_columns(self, conn: sqlite3.Connection, results: Dict[str, Any]):
        """
        Check specific critical columns that should never be null.

        Args:
            conn: Database connection
            results: Results dictionary to update with findings
        """
        cursor = conn.cursor()

        # Define critical columns that should never be null
        critical_checks = [
            # File table checks
            ("file", "uuid", "Primary key should never be NULL"),
            ("file", "file_name", "File name should never be NULL"),
            ("file", "course_code", "Course code should be set for all files"),

            # Chunks table checks
            ("chunks", "uuid", "Chunk UUID should never be NULL"),
            ("chunks", "file_uuid", "File reference should never be NULL"),
            ("chunks", "vector", "Embeddings missing - chunks without vectors"),

            # Check for orphaned records
            ("chunks", "file_uuid", "Orphaned chunks - file_uuid not in file table")
        ]

        for table, column, description in critical_checks:
            try:
                # Check if table exists
                cursor.execute(f"SELECT name FROM sqlite_master WHERE type='table' AND name=?", (table,))
                if not cursor.fetchone():
                    continue

                # Check if column exists
                cursor.execute(f"PRAGMA table_info({table})")
                columns = [col[1] for col in cursor.fetchall()]
                if column not in columns:
                    continue

                # Special check for orphaned chunks
                if table == "chunks" and column == "file_uuid" and description.startswith("Orphaned"):
                    cursor.execute("""
                        SELECT COUNT(*)
                        FROM chunks c
                        LEFT JOIN file f ON c.file_uuid = f.uuid
                        WHERE f.uuid IS NULL
                    """)
                    orphaned_count = cursor.fetchone()[0]
                    if orphaned_count > 0:
                        results["warnings"].append(f"CRITICAL: {orphaned_count} orphaned chunks found")
                else:
                    # Check for NULL values in critical column
                    cursor.execute(f"SELECT COUNT(*) FROM {table} WHERE {column} IS NULL")
                    null_count = cursor.fetchone()[0]

                    if null_count > 0:
                        severity = "CRITICAL" if column in ["uuid", "file_uuid"] else "WARNING"
                        results["warnings"].append(
                            f"{severity}: {description} - {null_count} NULL values in {table}.{column}"
                        )

            except Exception as e:
                logger.error(f"Error checking {table}.{column}: {str(e)}")

    def check_embedding_completeness(self) -> Dict[str, Any]:
        """
        Check embedding completeness for files and chunks.

        Returns:
            Dictionary with embedding statistics
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        try:
            results = {
                "files": {},
                "chunks": {},
                "issues": []
            }

            # Check file table embeddings
            cursor.execute("""
                SELECT COUNT(*) as total,
                       SUM(CASE WHEN vector IS NULL THEN 1 ELSE 0 END) as missing_vectors,
                       SUM(CASE WHEN vector IS NOT NULL THEN 1 ELSE 0 END) as has_vectors
                FROM file
            """)
            file_stats = cursor.fetchone()

            results["files"] = {
                "total": file_stats[0],
                "missing_embeddings": file_stats[1],
                "has_embeddings": file_stats[2],
                "completion_percentage": (file_stats[2] / file_stats[0] * 100) if file_stats[0] > 0 else 0
            }

            # Check chunks table embeddings
            cursor.execute("""
                SELECT COUNT(*) as total,
                       SUM(CASE WHEN vector IS NULL THEN 1 ELSE 0 END) as missing_vectors,
                       SUM(CASE WHEN vector IS NOT NULL THEN 1 ELSE 0 END) as has_vectors
                FROM chunks
            """)
            chunk_stats = cursor.fetchone()

            results["chunks"] = {
                "total": chunk_stats[0],
                "missing_embeddings": chunk_stats[1],
                "has_embeddings": chunk_stats[2],
                "completion_percentage": (chunk_stats[2] / chunk_stats[0] * 100) if chunk_stats[0] > 0 else 0
            }

            # Identify files with chunks but no file embedding
            cursor.execute("""
                SELECT f.file_name, f.course_code
                FROM file f
                WHERE f.vector IS NULL
                AND EXISTS (SELECT 1 FROM chunks c WHERE c.file_uuid = f.uuid)
                LIMIT 10
            """)
            files_missing_embeddings = cursor.fetchall()

            if files_missing_embeddings:
                results["issues"].append({
                    "type": "files_without_embeddings",
                    "description": "Files with chunks but missing file-level embeddings",
                    "sample": [{"file": f[0], "course": f[1]} for f in files_missing_embeddings],
                    "total_count": results["files"]["missing_embeddings"]
                })

            # Identify chunks without embeddings
            cursor.execute("""
                SELECT COUNT(DISTINCT file_uuid)
                FROM chunks
                WHERE vector IS NULL
            """)
            files_with_missing_chunk_embeddings = cursor.fetchone()[0]

            if files_with_missing_chunk_embeddings > 0:
                results["issues"].append({
                    "type": "chunks_without_embeddings",
                    "description": "Chunks missing embeddings",
                    "affected_files": files_with_missing_chunk_embeddings,
                    "total_chunks": results["chunks"]["missing_embeddings"]
                })

            return results

        finally:
            conn.close()

    def generate_report(self, output_path: Optional[str] = None) -> str:
        """
        Generate a comprehensive validation report.

        Args:
            output_path: Optional path to save the report

        Returns:
            Formatted report string
        """
        # Run all checks
        integrity_results = self.check_database_integrity()
        embedding_results = self.check_embedding_completeness()

        # Format report
        report_lines = [
            "=" * 70,
            "DATABASE VALIDATION REPORT",
            "=" * 70,
            f"Database: {self.db_path}",
            f"Generated: {integrity_results['check_timestamp']}",
            "",
            "SUMMARY",
            "-" * 40,
            f"Total Tables: {integrity_results['statistics']['total_tables']}",
            f"Empty Tables: {integrity_results['statistics']['empty_tables']}",
            f"Total Columns Checked: {integrity_results['statistics']['total_columns_checked']}",
            f"Completely NULL Columns: {integrity_results['statistics']['completely_null_columns']}",
            f"Tables with Issues: {integrity_results['statistics']['tables_with_issues']}",
            ""
        ]

        # Add NULL columns section
        if integrity_results["null_columns"]:
            report_lines.extend([
                "COMPLETELY NULL COLUMNS (CRITICAL)",
                "-" * 40
            ])
            for null_col in integrity_results["null_columns"]:
                report_lines.append(
                    f"  - {null_col['table']}.{null_col['column']} "
                    f"(Table has {null_col['row_count']} rows)"
                )
            report_lines.append("")

        # Add empty tables section
        if integrity_results["empty_tables"]:
            report_lines.extend([
                "EMPTY TABLES",
                "-" * 40
            ])
            for table in integrity_results["empty_tables"]:
                report_lines.append(f"  - {table}")
            report_lines.append("")

        # Add embedding statistics
        report_lines.extend([
            "EMBEDDING STATISTICS",
            "-" * 40,
            "Files:",
            f"  Total: {embedding_results['files']['total']}",
            f"  With Embeddings: {embedding_results['files']['has_embeddings']}",
            f"  Missing Embeddings: {embedding_results['files']['missing_embeddings']}",
            f"  Completion: {embedding_results['files']['completion_percentage']:.1f}%",
            "",
            "Chunks:",
            f"  Total: {embedding_results['chunks']['total']}",
            f"  With Embeddings: {embedding_results['chunks']['has_embeddings']}",
            f"  Missing Embeddings: {embedding_results['chunks']['missing_embeddings']}",
            f"  Completion: {embedding_results['chunks']['completion_percentage']:.1f}%",
            ""
        ])

        # Add warnings
        if integrity_results["warnings"]:
            report_lines.extend([
                "WARNINGS & ISSUES",
                "-" * 40
            ])
            for warning in integrity_results["warnings"]:
                report_lines.append(f"  ⚠️  {warning}")
            report_lines.append("")

        # Add embedding issues
        if embedding_results["issues"]:
            report_lines.extend([
                "EMBEDDING ISSUES",
                "-" * 40
            ])
            for issue in embedding_results["issues"]:
                report_lines.append(f"  • {issue['description']}")
                if issue["type"] == "chunks_without_embeddings":
                    report_lines.append(f"    Affected Files: {issue['affected_files']}")
                    report_lines.append(f"    Total Chunks: {issue['total_chunks']}")
            report_lines.append("")

        report_lines.extend([
            "=" * 70,
            "END OF REPORT",
            "=" * 70
        ])

        report = "\n".join(report_lines)

        # Save report if path provided
        if output_path:
            with open(output_path, 'w') as f:
                f.write(report)
            logger.info(f"Report saved to: {output_path}")

        return report


def validate_database(db_path: str, verbose: bool = True) -> Dict[str, Any]:
    """
    Quick validation function for database integrity.

    Args:
        db_path: Path to the database
        verbose: Whether to print the report

    Returns:
        Dictionary with validation results
    """
    validator = DatabaseValidator(db_path)

    # Run integrity check
    results = validator.check_database_integrity()

    # Get embedding status
    embedding_status = validator.check_embedding_completeness()

    # Combine results
    combined_results = {
        "integrity": results,
        "embeddings": embedding_status,
        "has_issues": len(results["null_columns"]) > 0 or len(results["empty_tables"]) > 0,
        "critical_issues": len([w for w in results["warnings"] if w.startswith("CRITICAL")]) > 0
    }

    if verbose:
        report = validator.generate_report()
        print(report)

    return combined_results


if __name__ == "__main__":
    # Example usage
    import sys

    if len(sys.argv) > 1:
        db_path = sys.argv[1]
    else:
        # Default path for testing
        db_path = "/path/to/collective_metadata.db"

    print(f"Validating database: {db_path}")

    # Run validation
    results = validate_database(db_path, verbose=True)

    # Exit with error code if critical issues found
    if results["critical_issues"]:
        sys.exit(1)