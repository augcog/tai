#!/usr/bin/env python3
"""
Script to populate the problem table from YAML metadata files.
"""

import json
import uuid
import yaml
from pathlib import Path
from sqlalchemy.orm import Session
from typing import Dict, Any, List, Optional
import sqlite3
import os



def generate_uuid() -> str:
    """Generate a new UUID string."""
    return str(uuid.uuid4())


def get_metadata_db():
    """Get connection to metadata database."""
    # Try multiple possible locations for the metadata database
    possible_paths = [
        "/home/bot/bot/yk/YK_final/course_yaml/metadata.db",  # collective database
        os.path.join(os.path.dirname(__file__), "..", "..", "ai_chatbot_backend", "db", "metadata.db"),
        "../metadata.db",  # fallback to current directory
        "metadata.db"  # fallback to current directory
    ]

    for db_path in possible_paths:
        if os.path.exists(db_path):
            return sqlite3.connect(db_path)

    # If no database found, use the collective database path (will be created if needed)
    return sqlite3.connect("/home/bot/bot/yk/YK_final/course_yaml/metadata.db")




def load_yaml_file(yaml_path: str) -> Dict[Any, Any]:
    """Load and parse a YAML file."""
    try:
        with open(yaml_path, 'r', encoding='utf-8') as file:
            return yaml.safe_load(file)
    except Exception as e:
        raise ValueError(f"Failed to load YAML file {yaml_path}: {e}")


def populate_problem_table_from_yaml(yaml_path: str, conn=None) -> int:
    """
    Populate the problem table from a YAML metadata file.

    Args:
        yaml_path: Path to the YAML metadata file
        conn: Optional database connection. If not provided, creates a new one.

    Returns:
        Number of problem records inserted

    Raises:
        ValueError: If YAML file is invalid or missing required fields
    """
    # Load YAML data
    yaml_data = load_yaml_file(yaml_path)

    # Extract file information
    file_uuid = yaml_data.get('file_uuid')
    if not file_uuid:
        raise ValueError("YAML file must contain 'file_uuid' field")

    # Get or create database connection
    close_conn = False
    if conn is None:
        conn = get_metadata_db()
        close_conn = True

    try:
        cursor = conn.cursor()
        inserted_count = 0

        # Process problems section
        problems = yaml_data.get('problems', [])
        for problem in problems:
            # Get problem metadata
            problem_index = problem.get('problem_index')
            problem_id = problem.get('problem_id')
            problem_content = problem.get('problem_content', '')

            # Process questions for this problem
            questions = problem.get('questions', {})
            for question_key, question_data in questions.items():
                # Extract question ID from key (e.g., 'question_1' -> '1')
                try:
                    question_id = question_key.split('_')[-1]
                except (ValueError, IndexError):
                    question_id = None

                # Determine question type from question data
                question_type = "regular"

                # Insert record using the correct table structure
                cursor.execute('''
                    INSERT INTO problem (
                        uuid, file_uuid, problem_index, problem_id, problem_content,
                        question_id, question, choices, answer, explanation, question_type
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''', (
                    generate_uuid(),
                    file_uuid,
                    problem_index,
                    problem_id,
                    problem_content,
                    question_id,
                    question_data.get('question', ''),
                    json.dumps(question_data.get('choices', [])),
                    json.dumps(question_data.get('answer', [])),
                    question_data.get('explanation', ''),
                    question_type
                ))

                inserted_count += 1

        # Process comprehensive questions section
        comprehensive_questions = yaml_data.get('comprehensive_questions', [])
        for comp_question in comprehensive_questions:
            # Determine question type
            question_type = "comprehensive"

            # Insert comprehensive question record
            question_id = comp_question.get('question_id')
            problem_id = f"comprehensive_{question_id}" if question_id else "comprehensive"

            cursor.execute('''
                INSERT INTO problem (
                    uuid, file_uuid, problem_index, problem_id, problem_content,
                    question_id, question, choices, answer, explanation, question_type
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                generate_uuid(),
                file_uuid,
                None,  # problem_index
                problem_id,
                '',    # problem_content
                question_id,
                comp_question.get('question_text', ''),
                json.dumps(comp_question.get('options', [])),
                json.dumps(comp_question.get('correct_answers', [])),
                comp_question.get('explanation', ''),
                question_type
            ))

            inserted_count += 1

        # Commit changes
        conn.commit()
        return inserted_count

    except Exception as e:
        conn.rollback()
        raise ValueError(f"Failed to insert problem data: {e}")
    finally:
        if close_conn:
            conn.close()


def populate_from_directory(directory_path: str, pattern: str = "*.yaml") -> Dict[str, int]:
    """
    Populate problem table from all YAML files in a directory.

    Args:
        directory_path: Directory containing YAML files
        pattern: File pattern to match (default: "*.yaml")

    Returns:
        Dictionary mapping file paths to number of records inserted
    """
    directory = Path(directory_path)
    if not directory.exists():
        raise ValueError(f"Directory does not exist: {directory_path}")

    results = {}
    yaml_files = list(directory.rglob(pattern))

    if not yaml_files:
        print(f"No YAML files found in {directory_path} matching pattern {pattern}")
        return results

    conn = get_metadata_db()
    try:
        for yaml_file in yaml_files:
            try:
                count = populate_problem_table_from_yaml(str(yaml_file), conn)
                results[str(yaml_file)] = count
                print(f"✓ Processed {yaml_file}: {count} records inserted")
            except Exception as e:
                print(f"✗ Failed to process {yaml_file}: {e}")
                results[str(yaml_file)] = 0
    finally:
        conn.close()

    return results


def clear_problem_table(conn=None) -> int:
    """
    Clear all records from the problem table.

    Args:
        conn: Optional database connection. If not provided, creates a new one.

    Returns:
        Number of records deleted
    """
    close_conn = False
    if conn is None:
        conn = get_metadata_db()
        close_conn = True

    try:
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM problem")
        count = cursor.fetchone()[0]
        cursor.execute("DELETE FROM problem")
        conn.commit()
        return count
    finally:
        if close_conn:
            conn.close()


def update_problems_from_yaml(yaml_path: str, conn=None) -> Dict[str, int]:
    """
    Update the problem table from a YAML metadata file.

    Compares YAML content with existing database records and:
    - Inserts new regular questions that don't exist
    - Updates existing questions if content has changed
    - Skips comprehensive questions (only handles regular questions)

    Args:
        yaml_path: Path to the YAML metadata file
        conn: Optional database connection. If not provided, creates a new one.

    Returns:
        Dictionary with update statistics: {'inserted': int, 'updated': int, 'skipped': int}

    Raises:
        ValueError: If YAML file is invalid or missing required fields
    """
    # Load YAML data
    yaml_data = load_yaml_file(yaml_path)

    # Extract file information
    file_uuid = yaml_data.get('file_uuid')
    if not file_uuid:
        raise ValueError("YAML file must contain 'file_uuid' field")

    # Get or create database connection
    close_conn = False
    if conn is None:
        conn = get_metadata_db()
        close_conn = True

    stats = {'inserted': 0, 'updated': 0, 'skipped': 0, 'errors': 0}

    try:
        cursor = conn.cursor()

        # Get existing problems for this file from database
        cursor.execute('''
            SELECT problem_index, question_id, problem_id, question, choices, answer, explanation, uuid
            FROM problem
            WHERE file_uuid = ? AND question_type = 'regular'
        ''', (file_uuid,))

        existing_problems = {}
        for row in cursor.fetchall():
            problem_index, question_id, problem_id_db, question, choices, answer, explanation, uuid = row
            # Use combination of problem_index and question_id for unique identification
            # This should be unique per file since question_id is extracted from question_1, question_2, etc.
            key = (problem_index, question_id)
            existing_problems[key] = {
                'uuid': uuid,
                'problem_id': problem_id_db,
                'question': question or '',
                'choices': choices or '',
                'answer': answer or '',
                'explanation': explanation or ''
            }


        # Process problems section from YAML (only regular questions)
        problems = yaml_data.get('problems', [])
        for problem in problems:
            # Get problem metadata
            problem_index = problem.get('problem_index')
            problem_id = problem.get('problem_id')
            problem_content = problem.get('problem_content', '')

            # Process questions for this problem
            questions = problem.get('questions', {})
            for question_key, question_data in questions.items():
                try:
                    # Extract question ID from key (e.g., 'question_1' -> '1')
                    # Keep as string to match database format
                    try:
                        question_id = question_key.split('_')[-1]
                    except (ValueError, IndexError):
                        question_id = None

                    # Prepare question data from YAML
                    yaml_question = question_data.get('question', '')
                    yaml_choices = json.dumps(question_data.get('choices', []))
                    yaml_answer = json.dumps(question_data.get('answer', []))
                    yaml_explanation = question_data.get('explanation', '')

                    # Check if this question already exists
                    key = (problem_index, question_id)

                    if key in existing_problems:
                        # Compare with existing data
                        existing = existing_problems[key]

                        if (existing['question'] != yaml_question or
                            existing['choices'] != yaml_choices or
                            existing['answer'] != yaml_answer or
                            existing['explanation'] != yaml_explanation):

                            # Update existing record
                            cursor.execute('''
                                UPDATE problem SET
                                    problem_content = ?, question = ?, choices = ?,
                                    answer = ?, explanation = ?
                                WHERE uuid = ?
                            ''', (
                                problem_content, yaml_question, yaml_choices,
                                yaml_answer, yaml_explanation, existing['uuid']
                            ))
                            stats['updated'] += 1
                        else:
                            stats['skipped'] += 1
                    else:
                        # Insert new record
                        cursor.execute('''
                            INSERT INTO problem (
                                uuid, file_uuid, problem_index, problem_id, problem_content,
                                question_id, question, choices, answer, explanation, question_type
                            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                        ''', (
                            generate_uuid(), file_uuid, problem_index, problem_id, problem_content,
                            question_id, yaml_question, yaml_choices, yaml_answer, yaml_explanation, 'regular'
                        ))
                        stats['inserted'] += 1

                except Exception as e:
                    print(f"Error processing question {question_key} in {yaml_path}: {e}")
                    stats['errors'] += 1
                    continue

        # Commit changes
        conn.commit()
        return stats

    except Exception as e:
        conn.rollback()
        raise ValueError(f"Failed to update problem data from {yaml_path}: {e}")
    finally:
        if close_conn:
            conn.close()


def update_problems_from_directory(directory_path: str, pattern: str = "*.yaml") -> Dict[str, Dict[str, int]]:
    """
    Update problem table from all YAML files in a directory.

    Args:
        directory_path: Directory containing YAML files
        pattern: File pattern to match (default: "*.yaml")

    Returns:
        Dictionary mapping file paths to update statistics
    """
    directory = Path(directory_path)
    if not directory.exists():
        raise ValueError(f"Directory does not exist: {directory_path}")

    results = {}
    yaml_files = list(directory.rglob(pattern))

    if not yaml_files:
        print(f"No YAML files found in {directory_path} matching pattern {pattern}")
        return results

    conn = get_metadata_db()
    try:
        total_stats = {'inserted': 0, 'updated': 0, 'skipped': 0, 'errors': 0, 'files_processed': 0}

        for yaml_file in yaml_files:
            try:
                stats = update_problems_from_yaml(str(yaml_file), conn)
                results[str(yaml_file)] = stats

                # Update totals
                for key in ['inserted', 'updated', 'skipped', 'errors']:
                    total_stats[key] += stats[key]
                total_stats['files_processed'] += 1

                # Print progress
                if stats['inserted'] > 0 or stats['updated'] > 0:
                    print(f"✓ {yaml_file.name}: +{stats['inserted']} new, ~{stats['updated']} updated, ={stats['skipped']} unchanged")
                else:
                    print(f"- {yaml_file.name}: no changes needed")

            except Exception as e:
                print(f"✗ Failed to process {yaml_file}: {e}")
                results[str(yaml_file)] = {'inserted': 0, 'updated': 0, 'skipped': 0, 'errors': 1}
                total_stats['errors'] += 1

        # Print summary
        print(f"\n=== UPDATE SUMMARY ===")
        print(f"Files processed: {total_stats['files_processed']}")
        print(f"Questions inserted: {total_stats['inserted']}")
        print(f"Questions updated: {total_stats['updated']}")
        print(f"Questions unchanged: {total_stats['skipped']}")
        print(f"Errors: {total_stats['errors']}")

    finally:
        conn.close()

    return results


if __name__ == "__main__":
    # Example usage - update problems from a specific directory
    import sys

    if len(sys.argv) > 1:
        directory_path = sys.argv[1]
    else:
        directory_path = "/home/bot/bot/yk/YK_final/courses/ROAR Academy"

    print(f"Updating problems from YAML files in: {directory_path}")
    try:
        results = update_problems_from_directory(directory_path)
        print(f"Successfully processed {len(results)} files")
    except Exception as e:
        print(f"Error: {e}")