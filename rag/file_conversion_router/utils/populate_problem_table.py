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
    db_path = os.path.join(os.path.dirname(__file__), "..", "..", "ai_chatbot_backend", "db", "metadata.db")
    if not os.path.exists(db_path):
        db_path = "../metadata.db"  # fallback to current directory
    return sqlite3.connect(db_path)




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
                # Extract question ID from key (e.g., 'question_1' -> 1)
                try:
                    question_id = int(question_key.split('_')[-1])
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


if __name__ == "__main__":
    populate_from_directory("/courses")