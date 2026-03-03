"""Service for managing sentence-to-bbox mappings in the database."""

import json
import logging
import re
import sqlite3
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple


def generate_sentence_mapping_from_json(lines_json_path: str) -> List[Dict[str, Any]]:
    """
    Load a *_lines.json file and extract ordered sentence-to-bbox mappings.

    Args:
        lines_json_path: Path to the lines JSON file (e.g., disc01.pdf_lines.json)

    Returns:
        List of dicts with sentence metadata:
        - content: str - Sentence text
        - bbox: [x1, y1, x2, y2] - Bounding box coordinates
        - index: int - Sequential sentence index
        - page_index: int - PDF page number (0-indexed)
        - block_type: str - Block type (e.g., "title", "text", "code")

    Example:
        >>> mapping = generate_sentence_mapping_from_json("disc01.pdf_lines.json")
        >>> len(mapping)
        52
        >>> mapping[0]
        {"content": "While and If", "bbox": [51, 116, 154, 146],
         "index": 0, "page_index": 0, "block_type": "title"}
    """
    lines_path = Path(lines_json_path)

    if not lines_path.exists():
        raise FileNotFoundError(f"Lines JSON file not found: {lines_json_path}")

    try:
        with lines_path.open('r', encoding='utf-8') as f:
            lines_data = json.load(f)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in {lines_json_path}: {e}")

    if not isinstance(lines_data, list):
        raise ValueError(f"Expected list in {lines_json_path}, got {type(lines_data)}")

    sentence_mapping = []

    for item in lines_data:
        # Each item should have spans array
        spans = item.get('spans', [])
        if not spans:
            continue

        # Get the first span (each item typically has one span after merging)
        span = spans[0]
        content = span.get('content', '').strip()
        bbox = span.get('bbox', [])

        if not content:
            continue

        # Validate bbox format [x1, y1, x2, y2]
        if not isinstance(bbox, list) or len(bbox) != 4:
            logging.warning(f"Invalid bbox format for content: {content[:50]}...")
            continue

        sentence_mapping.append({
            'content': content,
            'bbox': bbox,
            'index': item.get('index'),
            'page_index': item.get('page_index'),
            'block_type': item.get('block_type')
        })

    logging.info(f"Generated sentence mapping with {len(sentence_mapping)} sentences from {lines_json_path}")
    return sentence_mapping


def update_file_extra_info_with_mapping(
    conn: sqlite3.Connection,
    file_uuid: str,
    sentence_mapping: List[Dict[str, Any]]
) -> None:
    """
    Update the extra_info column of a file record with sentence mapping.
    Merges with existing extra_info if present.

    Args:
        conn: SQLite database connection
        file_uuid: UUID of the file record to update
        sentence_mapping: List of dicts with keys: content, bbox, index, page_index, block_type
    """
    # Get current extra_info
    row = conn.execute("SELECT extra_info FROM file WHERE uuid=?", (file_uuid,)).fetchone()

    if not row:
        raise ValueError(f"File with uuid {file_uuid} not found in database")

    # Parse existing extra_info or create new dict
    extra_info_str = row[0]
    if extra_info_str:
        try:
            extra_info = json.loads(extra_info_str)
        except json.JSONDecodeError:
            logging.warning(f"Could not parse existing extra_info for {file_uuid}, creating new")
            extra_info = {}
    else:
        extra_info = {}

    # Add/update sentence_mapping
    extra_info['sentence_mapping'] = sentence_mapping

    # Update database
    updated_extra_info = json.dumps(extra_info, ensure_ascii=False)
    conn.execute(
        "UPDATE file SET extra_info=?, update_time=datetime('now', 'localtime') WHERE uuid=?",
        (updated_extra_info, file_uuid)
    )
    conn.commit()

    logging.info(f"Updated extra_info for file {file_uuid} with {len(sentence_mapping)} sentences")


def find_file_uuid(
    conn: sqlite3.Connection,
    file_name: str,
    course_code: str
) -> Optional[str]:
    """
    Find file UUID by file name and course code.

    Args:
        conn: SQLite database connection
        file_name: Name of the file (e.g., "disc01.pdf")
        course_code: Course code (e.g., "CS61A")

    Returns:
        File UUID if found, None otherwise
    """
    row = conn.execute(
        "SELECT uuid FROM file WHERE file_name=? AND course_code=?",
        (file_name, course_code)
    ).fetchone()

    if row:
        return row[0]
    return None


def add_sentence_mapping_to_file(
    db_path: str,
    lines_json_path: str,
    file_name: str,
    course_code: str
) -> bool:
    """
    Complete workflow to add sentence mapping to a file in the database.

    Args:
        db_path: Path to SQLite database
        lines_json_path: Path to lines JSON file
        file_name: Name of the PDF file
        course_code: Course code

    Returns:
        True if successful, False otherwise
    """
    try:
        # Generate mapping from JSON
        sentence_mapping = generate_sentence_mapping_from_json(lines_json_path)

        if not sentence_mapping:
            logging.error("No sentences found in JSON file")
            return False

        # Connect to database
        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row

        # Find file record
        file_uuid = find_file_uuid(conn, file_name, course_code)

        if not file_uuid:
            logging.error(f"File not found in database: {file_name} (course: {course_code})")
            conn.close()
            return False

        # Update extra_info
        update_file_extra_info_with_mapping(conn, file_uuid, sentence_mapping)

        conn.close()
        logging.info(f"Successfully added sentence mapping for {file_name}")
        return True

    except Exception as e:
        logging.error(f"Error adding sentence mapping: {e}")
        return False


def generate_lines_json_from_middle_json(
    middle_json_path: str,
    output_lines_path: str
) -> bool:
    """
    Convert MinerU's _middle.json to _lines.json format for sentence mapping.
    Extracts line information and merges into sentence-based spans.

    Args:
        middle_json_path: Path to the _middle.json file from MinerU
        output_lines_path: Path where _lines.json should be saved

    Returns:
        True if successful, False otherwise
    """
    def merge_bboxes(bboxes: List[List[int]]) -> List[int]:
        """Merge multiple bboxes into one encompassing bbox."""
        if not bboxes:
            return [0, 0, 0, 0]
        x1 = min(bbox[0] for bbox in bboxes)
        y1 = min(bbox[1] for bbox in bboxes)
        x2 = max(bbox[2] for bbox in bboxes)
        y2 = max(bbox[3] for bbox in bboxes)
        return [x1, y1, x2, y2]

    def combine_line_content(line: Dict[str, Any]) -> Tuple[str, List[List[int]]]:
        """Extract combined text and all bboxes from a line's spans."""
        texts = []
        bboxes = []
        for span in line.get('spans', []):
            content = span.get('content', '').strip()
            if content:
                texts.append(content)
                if 'bbox' in span:
                    bboxes.append(span['bbox'])
        return ' '.join(texts), bboxes

    def ends_with_complete_sentence(text: str) -> bool:
        """Check if text ends with complete sentence."""
        text = text.strip()
        return len(text) > 0 and text[-1] in '.!?'

    def calculate_font_size(bbox: List[int]) -> int:
        """Estimate font size from bbox height."""
        return bbox[3] - bbox[1]

    def calculate_vertical_gap(bbox1: List[int], bbox2: List[int]) -> int:
        """Calculate vertical gap between two bboxes."""
        return bbox2[1] - bbox1[3]

    def should_merge_with_next(
        current_text: str,
        first_line_bboxes: List[List[int]],
        all_bboxes: List[List[int]],
        next_bboxes: List[List[int]],
        block_type: str,
        lines_merged: int
    ) -> bool:
        """Decide if we should merge current text with next line."""
        if block_type == "title" or lines_merged >= 10:
            return False
        if ends_with_complete_sentence(current_text):
            return False
        if not first_line_bboxes or not all_bboxes or not next_bboxes:
            return True

        font_size = min(calculate_font_size(bbox) for bbox in first_line_bboxes)
        current_bbox = merge_bboxes(all_bboxes)
        next_bbox = merge_bboxes(next_bboxes)
        vertical_gap = calculate_vertical_gap(current_bbox, next_bbox)

        if vertical_gap > 2 * font_size:
            return False
        return True

    def find_sentence_boundaries(text: str) -> List[int]:
        """Find positions where sentences end in text."""
        pattern = r'([.!?])\s+(?=[A-Z])|([.!?])\s*$'
        boundaries = []
        for match in re.finditer(pattern, text):
            boundaries.append(match.end())
        return boundaries

    def split_into_sentence_spans(text: str, bboxes: List[List[int]]) -> List[Tuple[str, List[int]]]:
        """Split text into sentences and assign merged bbox to each."""
        boundaries = find_sentence_boundaries(text)
        if not boundaries:
            return [(text.strip(), merge_bboxes(bboxes))]

        sentences = []
        start = 0
        for end in boundaries:
            sentence = text[start:end].strip()
            if sentence:
                sentences.append((sentence, merge_bboxes(bboxes)))
            start = end

        if start < len(text):
            remaining = text[start:].strip()
            if remaining:
                sentences.append((remaining, merge_bboxes(bboxes)))

        return sentences if sentences else [(text.strip(), merge_bboxes(bboxes))]

    def merge_lines_into_sentences(lines: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Merge lines so that each resulting line contains complete sentences."""
        if not lines:
            return lines

        result = []
        i = 0

        while i < len(lines):
            current_line = lines[i]
            page_index = current_line['page_index']
            block_type = current_line['block_type']

            text, bboxes = combine_line_content(current_line)

            if not text:
                i += 1
                continue

            first_line_bboxes = bboxes[:]
            j = i + 1
            lines_merged = 1

            while j < len(lines):
                next_line = lines[j]

                if (next_line['page_index'] != page_index or
                    next_line['block_type'] != block_type):
                    break

                next_text, next_bboxes = combine_line_content(next_line)
                if not next_text:
                    j += 1
                    continue

                if not should_merge_with_next(text, first_line_bboxes, bboxes, next_bboxes, block_type, lines_merged):
                    break

                text = text + ' ' + next_text
                bboxes.extend(next_bboxes)
                lines_merged += 1
                j += 1

            sentence_spans = split_into_sentence_spans(text, bboxes)

            for span_content, span_bbox in sentence_spans:
                result.append({
                    'index': len(result),
                    'page_index': page_index,
                    'block_type': block_type,
                    'spans': [{
                        'bbox': span_bbox,
                        'content': span_content,
                        'type': 'text'
                    }]
                })

            i = j

        return result

    # Main conversion logic
    try:
        middle_path = Path(middle_json_path)
        if not middle_path.exists():
            logging.error(f"Middle JSON file not found: {middle_json_path}")
            return False

        # Load middle JSON
        with middle_path.open('r', encoding='utf-8') as f:
            json_data = json.load(f)

        # Extract lines from JSON structure
        extracted_lines = []
        pdf_info = json_data.get('pdf_info', [])

        for page_idx, page in enumerate(pdf_info):
            preproc_blocks = page.get('preproc_blocks', [])

            for block in preproc_blocks:
                block_type = block.get('type', 'unknown')
                lines = block.get('lines', [])

                for line in lines:
                    line_info = {
                        'page_index': page_idx,
                        'block_type': block_type,
                        **{k: v for k, v in line.items() if k not in ['bbox', 'index']}
                    }

                    if 'spans' in line_info:
                        cleaned_spans = []
                        for span in line_info['spans']:
                            cleaned_span = {k: v for k, v in span.items() if k != 'score'}
                            cleaned_spans.append(cleaned_span)
                        line_info['spans'] = cleaned_spans

                    extracted_lines.append(line_info)

        # Merge lines into sentence-based spans
        sentence_lines = merge_lines_into_sentences(extracted_lines)

        # Save to output file
        output_path = Path(output_lines_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with output_path.open('w', encoding='utf-8') as f:
            json.dump(sentence_lines, f, ensure_ascii=False, indent=2)

        logging.info(f"Generated lines JSON with {len(sentence_lines)} sentences: {output_lines_path}")
        return True

    except Exception as e:
        logging.error(f"Error generating lines JSON from middle JSON: {e}")
        return False
