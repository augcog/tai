"""Internal service to process a directory of files and schedule conversion tasks for each file."""

import logging
import uuid
import hashlib
import pathlib
# from concurrent.futures import as_completed
from pathlib import Path
from typing import Dict, Type, Union
import yaml
import os
import sqlite3
from typing import List, Tuple
from pathspec import PathSpec
from file_conversion_router.conversion.base_converter import BaseConverter
from file_conversion_router.conversion.ed_converter import EdConverter
from file_conversion_router.conversion.html_converter import HtmlConverter
from file_conversion_router.conversion.md_converter import MarkdownConverter
from file_conversion_router.conversion.notebook_converter import NotebookConverter
from file_conversion_router.conversion.pdf_converter import PdfConverter
from file_conversion_router.conversion.python_converter import PythonConverter
from file_conversion_router.conversion.rst_converter import RstConverter
from file_conversion_router.conversion.video_converter import VideoConverter
from file_conversion_router.utils.logger import content_logger, set_log_file_path
from file_conversion_router.classes.chunk import Chunk
import hashlib
import json
from typing import Dict, Any


ConverterMapping = Dict[str, Type[BaseConverter]]

# Mapping from file extensions to their corresponding conversion classes
converter_mapping: ConverterMapping = {
    ".pdf": PdfConverter,
    ".md": MarkdownConverter,
    ".rst": RstConverter,
    ".mp4": VideoConverter,
    # ".json": EdConverter,
    ".html": HtmlConverter,
    ".ipynb": NotebookConverter,
    ".py": PythonConverter,
    '.mkv': VideoConverter,
    '.webm': VideoConverter,
    '.mov': VideoConverter,
    #     TODO: Add more file types and converters here
}


def process_folder(
        input_dir: Union[str, Path],
        output_dir: Union[str, Path],
        course_name: str,
        course_code: str,
        log_dir: Union[str, Path] = None,
        db_path: Union[str, Path] = None,
) -> None:
    """Walk through the input directory and schedule conversion tasks for specified file types."""
    logging.getLogger().setLevel(logging.INFO)
    output_dir = Path(output_dir)
    input_dir = Path(input_dir)

    conn = ensure_chunk_db(Path(db_path))

    if log_dir:
        set_log_file_path(content_logger, log_dir)

    if not input_dir.is_dir():
        raise ValueError(f"Provided input path {input_dir} is not a directory.")

    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)
    elif not output_dir.is_dir():
        raise ValueError(f"Provided output path {output_dir} is not a directory.")

    valid_extensions = tuple(converter_mapping.keys())

    ignore_file = '../file_conversion_router/services/.conversionignore'
    patterns = _load_patterns(ignore_file)
    spec = PathSpec.from_lines("gitwildmatch", patterns)

    def should_ignore(file_path: Path) -> bool:
        relative_path = str(file_path.relative_to(input_dir))
        matched = spec.match_file(relative_path)
        if matched:
            logging.info(f"Ignoring file: {relative_path} based on ignore patterns.")
        return matched

    for input_file_path in input_dir.rglob("*"):
        if not (input_file_path.is_file() and input_file_path.suffix in valid_extensions):
            continue
        if should_ignore(input_file_path):
            continue

        fhash = file_hash_for_cache(input_file_path)
        file_record = get_file_record_by_hash(conn, fhash)

        if file_record:
            file_uuid_cached = file_record["uuid"]
            current_relative_path = str(input_file_path.relative_to(input_dir.parent))

            # Update file path if it has changed and old file no longer exists
            path_updated = update_file_path_if_changed(conn, file_uuid_cached, current_relative_path, input_dir.parent)
            if path_updated:
                conn.commit()

            logging.info(f"[SKIP cache-hit] {input_file_path} already ingested as file_uuid={file_uuid_cached}")
            continue

        output_subdir = output_dir / input_file_path.relative_to(input_dir).parent
        output_subdir.mkdir(parents=True, exist_ok=True)
        output_file_path = output_subdir / input_file_path.stem

        converter_class = converter_mapping.get(input_file_path.suffix)
        if not converter_class:
            logging.warning(f"No converter available for file type {input_file_path.suffix}")
            continue

        file_uuid = deterministic_file_uuid(fhash)
        converter = converter_class(course_name, course_code, file_uuid)
        (chunks, metadata), _ = converter.convert(input_file_path, output_file_path, input_dir.parent)
        if not chunks:
            chunks = []

        fp = chunks[0].file_path if chunks and getattr(chunks[0], "file_path", None) else input_file_path.relative_to(input_dir.parent)
        relative_path = str(fp)

        # sections from metadata if provided
        sections = []
        url = ""
        comprehensive_questions = []
        if isinstance(metadata, dict):
            sections = metadata.get("sections", []) or []
            comprehensive_questions = metadata.get("comprehensive_questions", []) or []
            url = metadata.get("URL", "")

        # read extra info from json file if present
        extra_info = []
        extra_info_file = output_file_path / f"{input_file_path.name}.json"
        if extra_info_file.exists():
            try:
                with extra_info_file.open("r", encoding="utf-8") as f:
                    extra_info = json.load(f)
            except json.JSONDecodeError as e:
                logging.error(f"Failed to read extra info from {extra_info_file}: {e}")
                extra_info = []
        # write chunks + file row
        file_uuid_written = write_chunks_to_db(
            conn=conn,
            file_hash=fhash,
            relative_path=relative_path,
            file_name=input_file_path.name,
            chunks=chunks,
            course_code=course_code,
            course_name=course_name,
            sections=sections,
            extra_info=extra_info,
            url=url,
        )

        # upsert problems if present
        if isinstance(metadata, dict):
            problems = metadata.get("problems") or metadata.get("problem_set") or []
        else:
            problems = []

        if problems:
            conn.execute("BEGIN IMMEDIATE")
            for pr in problems:
                q_container = pr.get("questions") or pr.get("question_set") or []
                for q_id, q in _iter_questions_local(q_container):
                    upsert_problem_meta(conn, file_uuid_written, pr, q_id, q)
            conn.commit()
        
        # upsert comprehensive questions if present
        if comprehensive_questions:
            conn.execute("BEGIN IMMEDIATE")
            for idx, cq in enumerate(comprehensive_questions, start=1):
                # Create a problem entry for each comprehensive question
                # Treat it as a special type of problem with comprehensive question data
                problem_data = {
                    "problem_index": idx,
                    "problem_id": f"comprehensive_{idx}",
                    "problem_content": cq.get("question_text", "")
                }
                
                # Handle questions within comprehensive questions
                q_container = cq.get("questions") or cq.get("question_set") or []
                if q_container:
                    for q_id, q in _iter_questions_local(q_container):
                        upsert_problem_meta(conn, file_uuid_written, problem_data, q_id, q, "comprehensive")
                else:
                    # If no nested questions, store the comprehensive question itself
                    question_data = {
                        "question": cq.get("question", "") or cq.get("text", ""),
                        "choices": cq.get("options"),
                        "answer": cq.get("correct_answers"),
                        "explanation": cq.get("explanation")
                    }
                    upsert_problem_meta(conn, file_uuid_written, problem_data, idx, question_data, "comprehensive")
            conn.commit()

        logging.info(f"[OK] {input_file_path} → {output_file_path} ({len(chunks)} chunks)")
        logging.info(f"Scheduled conversion for {input_file_path} to {output_file_path}")

    # Cleanup deleted files from database
    deleted_count = cleanup_deleted_files(conn, course_code, input_dir.parent)
    if deleted_count > 0:
        logging.info(f"Database cleanup completed: {deleted_count} deleted file(s) removed")

    conn.close()
    content_logger.info(f"Completed content checking for directory: {input_dir}")
    logging.info(f"Completed processing for directory: {input_dir}")


def cleanup_deleted_files(conn: sqlite3.Connection, course_code: str, input_dir_parent: Path) -> int:
    """
    Remove database entries for files that no longer exist on disk.
    Returns the number of files cleaned up.
    """
    deleted_count = 0

    # Get all files for this course from database
    rows = conn.execute(SQL_SELECT_FILES_BY_COURSE, (course_code,)).fetchall()

    for row in rows:
        file_uuid, relative_path, file_name = row
        file_path = input_dir_parent / relative_path

        # Check if file still exists on disk
        if not file_path.exists():
            # File has been deleted, remove from database
            conn.execute("BEGIN IMMEDIATE")
            conn.execute(SQL_DELETE_FILE_CASCADE, (file_uuid,))
            conn.commit()

            deleted_count += 1
            logging.info(f"[CLEANUP] Removed deleted file from database: {relative_path} (uuid={file_uuid})")

    if deleted_count > 0:
        logging.info(f"[CLEANUP] Removed {deleted_count} deleted file(s) from database")

    return deleted_count


def _load_patterns(ignore_file=None,
                   extra_patterns=None) -> List[str]:
    with open(ignore_file, "r", encoding="utf-8") as f:
        data = f.read()
    return data.splitlines() if ignore_file else [] + (extra_patterns or [])

def ensure_chunk_db(database_path) -> sqlite3.Connection:
    p = Path(database_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if not os.access(p.parent, os.W_OK):
        raise PermissionError(f"Parent directory not writable: {p.parent}")
    conn = sqlite3.connect(str(p))
    conn.row_factory = sqlite3.Row
    conn.executescript(SQL_INIT)  # single DB with tables: file, chunks, problem
    return conn

def upsert_file_meta(conn: sqlite3.Connection, f: dict):
    """
    Upsert into `file` using uuid as the conflict key.
    Expects keys: uuid, file_hash, sections, relative_path, course_code, course_name, file_name
    """
    args = (
        f["uuid"],
        f["file_hash"],
        json.dumps(f.get("sections", []), ensure_ascii=False) if not isinstance(f.get("sections"), str) else f["sections"],
        str(f.get("relative_path", f.get("file_path", ""))),
        f.get("course_code", ""),
        f.get("course_name"),
        f.get("file_name"),
        json.dumps(f.get("extra_info", {}), ensure_ascii=False) if f.get("extra_info") else None,
        f.get("url"),
    )
    conn.execute(SQL_UPSERT_FILE, args)


def upsert_problem_meta(conn: sqlite3.Connection, file_uuid: str, pr: dict, q_id: int | str, q: dict, question_type: str = "regular"):
    conn.execute(
        SQL_UPSERT_PROBLEM,
        (
            str(uuid.uuid4()),                # problem.uuid (row PK)
            file_uuid,                        # FK → file.file_uuid
            pr.get("problem_index"),
            pr.get("problem_id"),
            pr.get("problem_content"),
            q_id,
            q.get("question"),
            json.dumps(q.get("choices"), ensure_ascii=False),
            json.dumps(q.get("answer"), ensure_ascii=False),
            q.get("explanation"),
            question_type,
        )
    )

def file_content_hash(p: Path) -> str:
    h = hashlib.blake2b(digest_size=32)
    with p.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _blake2b_hex(s: str) -> str:
    h = hashlib.blake2b(digest_size=32)
    h.update(s.encode("utf-8"))
    return h.hexdigest()

def file_hash_for_cache(p: Path) -> str:
    """ Generate a stable hash for a file using content + file extension.
    For empty files, includes filename for individual tracking.
    This is location-independent, so moving files won't cause reprocessing.
    """
    extension = p.suffix.lower()
    filename = p.name

    try:
        if p.stat().st_size == 0:
            # For empty files, use extension + filename for individual tracking
            return _blake2b_hex(f"EMPTY|{extension}|{filename}")
        else:
            # For non-empty files, combine content hash with extension
            content_hash = file_content_hash(p)
            return _blake2b_hex(f"{content_hash}|{extension}")
    except Exception as e:
        # Fallback for files that can't be read
        return _blake2b_hex(f"ERROR|{extension}|{filename}|{str(e)}")


def write_chunks_to_db(conn: sqlite3.Connection,
                       file_hash: str,
                       relative_path: str,
                       file_name: str,
                       chunks: List[Chunk],
                       course_code: str | None = None,
                       course_name: str | None = None,
                       sections: list | str | None = None,
                       extra_info: list = None,
                       url: str = None) -> str:
    """
    Write chunks and bind file row. Returns file_uuid.
    """
    if not chunks:
        file_uuid = deterministic_file_uuid(file_hash)
    else:
        file_uuid = chunks[0].file_uuid
        if not file_uuid:
            raise ValueError("file_uuid must be set on chunks")
        for ch in chunks:
            if ch.file_uuid != file_uuid:
                raise ValueError(
                    f"All chunks must have the same file_uuid; "
                    f"chunk[0].file_uuid={file_uuid}, chunk[i].file_uuid={ch.file_uuid}"
                )

    # Enforce stable hash→uuid binding then upsert file row
    conn.execute("BEGIN IMMEDIATE")
    row = conn.execute(SQL_SELECT_FILE_UUID_BY_HASH, (file_hash,)).fetchone()
    if row:
        existing_uuid = row[0]
        if existing_uuid != file_uuid:
            raise ValueError(
                f"file_hash {file_hash} is already bound to file_uuid {existing_uuid}, "
                f"but chunks carry file_uuid {file_uuid}"
            )
    upsert_file_meta(conn, {
        "uuid": file_uuid,
        "file_hash": file_hash,
        "sections": sections or [],
        "relative_path": relative_path,
        "course_code": course_code or "",
        "course_name": course_name or "",
        "file_name": file_name,
        "extra_info": extra_info,
        "url": url or "",
    })

    # Upsert chunks
    for i, ch in enumerate(chunks):
        idx = ch.index
        text = ch.content
        if not isinstance(text, str):
            raise ValueError(f"chunk[{i}] content must be str")

        title_json = dump_title_list(ch.titles)


        chunk_url = getattr(ch, "chunk_url", None)
        fp = str(ch.file_path)
        ref_path = str(getattr(ch, "reference_path", None))
        c_name = getattr(ch, "course_name", course_name)
        c_code = getattr(ch, "course_code", course_code)
        c_idx = getattr(ch, "chunk_index", None)

        conn.execute(
            SQL_UPSERT_CHUNK,
            (ch.chunk_uuid, file_uuid, idx, text, title_json, chunk_url, fp, ref_path, c_name, c_code, c_idx),
        )

    conn.commit()
    return file_uuid

def dump_title_list(titles):
    if isinstance(titles, tuple):
        titles = list(titles)
    elif not isinstance(titles, list):
        titles = [titles]
    titles = [x if isinstance(x, str) else str(x) for x in titles]
    return json.dumps(titles, ensure_ascii=False)

def deterministic_file_uuid(file_hash: str) -> str:
    """
    Stable file_uuid derived from file content hash.
    Idempotent across runs/machines as long as file_hash is the same.
    """
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"file:{file_hash}"))

def get_file_record_by_hash(conn: sqlite3.Connection, file_hash: str) -> dict | None:
    """Get full file record by hash."""
    row = conn.execute("SELECT uuid, relative_path, file_name FROM file WHERE file_hash=?", (file_hash,)).fetchone()
    if row:
        return {
            "uuid": row[0],
            "relative_path": row[1],
            "file_name": row[2]
        }
    return None

def update_file_path_if_changed(conn: sqlite3.Connection, file_uuid: str, current_relative_path: str, input_dir_parent: Path) -> bool:
    """Update file path in database if it has changed and old file no longer exists. Returns True if updated."""
    row = conn.execute("SELECT relative_path FROM file WHERE uuid=?", (file_uuid,)).fetchone()
    if row and row[0] != current_relative_path:
        old_relative_path = row[0]
        old_file_path = input_dir_parent / old_relative_path

        # Only update if the old file no longer exists at its original location
        if not old_file_path.exists():
            conn.execute("UPDATE file SET relative_path=? WHERE uuid=?", (current_relative_path, file_uuid))
            logging.info(f"Updated file path for {file_uuid}: {old_relative_path} → {current_relative_path} (old file no longer exists)")
            return True
        else:
            # Old file still exists, this is a different file with same hash - should not happen normally
            logging.warning(f"File with same hash found at different path, but old file still exists: {old_relative_path} vs {current_relative_path}")
            return False
    return False

def _iter_questions_local(questions) -> list[tuple[int, dict]]:
    out = []
    if isinstance(questions, list):
        for i, q in enumerate(questions, start=1):
            out.append((i, q))
    elif isinstance(questions, dict):
        def _k(k):
            ks = str(k)
            return (0, int(ks)) if ks.isdigit() else (1, ks)

        for k in sorted(questions.keys(), key=_k):
            qid = int(k) if str(k).isdigit() else k
            out.append((qid, questions[k]))
    return out


SQL_INIT = """
PRAGMA foreign_keys=ON;
PRAGMA journal_mode=WAL;
PRAGMA synchronous=NORMAL;

CREATE TABLE IF NOT EXISTS file (
  uuid         TEXT PRIMARY KEY,
  file_hash    TEXT NOT NULL UNIQUE,
  sections     TEXT,
  relative_path TEXT,
  course_code  TEXT,
  course_name  TEXT,
  file_name    TEXT,
  extra_info   TEXT,
  url          TEXT
);

CREATE TABLE IF NOT EXISTS chunks (
  chunk_uuid     TEXT PRIMARY KEY,
  file_uuid      TEXT NOT NULL,
  idx            INTEGER NOT NULL,
  text           TEXT NOT NULL,
  title          TEXT,
  url            TEXT,
  file_path      TEXT,
  reference_path TEXT,
  course_name    TEXT,
  course_code    TEXT,
  chunk_index    INTEGER,
  FOREIGN KEY (file_uuid) REFERENCES file(uuid) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS problem (
  uuid            TEXT PRIMARY KEY,
  file_uuid       TEXT NOT NULL,
  problem_index   INTEGER,
  problem_id      TEXT,
  problem_content TEXT,
  question_id     TEXT,
  question        TEXT,
  choices         TEXT,
  answer          TEXT,
  explanation     TEXT,
  question_type   TEXT DEFAULT 'regular',
  FOREIGN KEY (file_uuid) REFERENCES file(uuid) ON DELETE CASCADE
);
"""
SQL_UPSERT_FILE = """
INSERT INTO file (uuid, file_hash, sections, relative_path, course_code, course_name, file_name, extra_info, url)
VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
ON CONFLICT(uuid) DO UPDATE SET
  file_hash     = excluded.file_hash,
  sections      = excluded.sections,
  relative_path = excluded.relative_path,
  course_code   = excluded.course_code,
  course_name   = excluded.course_name,
  file_name     = excluded.file_name,
  extra_info    = excluded.extra_info,
  url           = excluded.url;
"""


SQL_UPSERT_CHUNK = """
INSERT INTO chunks (
  chunk_uuid, file_uuid, idx, text, title, url, file_path, reference_path,
  course_name, course_code, chunk_index
) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
ON CONFLICT(chunk_uuid) DO UPDATE SET
  file_uuid      = excluded.file_uuid,
  idx            = excluded.idx,
  text           = excluded.text,
  title          = excluded.title,
  url            = excluded.url,
  file_path      = excluded.file_path,
  reference_path = excluded.reference_path,
  course_name    = excluded.course_name,
  course_code    = excluded.course_code,
  chunk_index    = excluded.chunk_index;
"""

SQL_UPSERT_PROBLEM = """
INSERT INTO problem (
  uuid, file_uuid, problem_index, problem_id, problem_content,
  question_id, question, choices, answer, explanation, question_type
) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
ON CONFLICT(uuid) DO UPDATE SET
  file_uuid       = excluded.file_uuid,
  problem_index   = excluded.problem_index,
  problem_id      = excluded.problem_id,
  problem_content = excluded.problem_content,
  question_id     = excluded.question_id,
  question        = excluded.question,
  choices         = excluded.choices,
  answer          = excluded.answer,
  explanation     = excluded.explanation,
  question_type   = excluded.question_type;
"""

SQL_SELECT_FILE_UUID_BY_HASH = "SELECT uuid FROM file WHERE file_hash=?;"

SQL_SELECT_FILES_BY_COURSE = "SELECT uuid, relative_path, file_name FROM file WHERE course_code=?;"

SQL_DELETE_FILE_CASCADE = "DELETE FROM file WHERE uuid=?;"
