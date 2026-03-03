#!/usr/bin/env python3
"""
CLI tool to add sentence-to-bbox mappings to the database.

Usage:
    python add_sentence_mapping.py \
        --lines-json tests/disc01/disc01.pdf_lines.json \
        --db tests/disc01/metadata.db \
        --file-name disc01.pdf \
        --course CS61A
"""

import argparse
import logging
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from file_conversion_router.services.sentence_mapping_service import (
    add_sentence_mapping_to_file
)


def setup_logging(verbose: bool = False):
    """Configure logging for the script."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )


def main():
    parser = argparse.ArgumentParser(
        description='Add sentence-to-bbox mapping to file table in database',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Add mapping for disc01.pdf
  python add_sentence_mapping.py \\
      --lines-json tests/disc01/disc01.pdf_lines.json \\
      --db tests/disc01/metadata.db \\
      --file-name disc01.pdf \\
      --course CS61A

  # With verbose logging
  python add_sentence_mapping.py \\
      --lines-json disc01.pdf_lines.json \\
      --db metadata.db \\
      --file-name disc01.pdf \\
      --course CS61A \\
      --verbose
        """
    )

    parser.add_argument(
        '--lines-json',
        required=True,
        help='Path to the lines JSON file (e.g., disc01.pdf_lines.json)'
    )
    parser.add_argument(
        '--db',
        required=True,
        help='Path to the SQLite database file'
    )
    parser.add_argument(
        '--file-name',
        required=True,
        help='Name of the PDF file in database (e.g., disc01.pdf)'
    )
    parser.add_argument(
        '--course',
        required=True,
        help='Course code (e.g., CS61A)'
    )
    parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        help='Enable verbose logging'
    )

    args = parser.parse_args()

    # Setup logging
    setup_logging(args.verbose)

    # Validate file paths
    lines_json_path = Path(args.lines_json)
    if not lines_json_path.exists():
        logging.error(f"Lines JSON file not found: {args.lines_json}")
        return 1

    db_path = Path(args.db)
    if not db_path.exists():
        logging.error(f"Database file not found: {args.db}")
        return 1

    # Execute the mapping addition
    logging.info(f"Adding sentence mapping for {args.file_name} in course {args.course}")
    logging.info(f"Source: {args.lines_json}")
    logging.info(f"Database: {args.db}")

    success = add_sentence_mapping_to_file(
        db_path=str(db_path),
        lines_json_path=str(lines_json_path),
        file_name=args.file_name,
        course_code=args.course
    )

    if success:
        logging.info("✓ Sentence mapping added successfully!")
        return 0
    else:
        logging.error("✗ Failed to add sentence mapping")
        return 1


if __name__ == '__main__':
    sys.exit(main())
