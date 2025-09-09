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

        fhash = file_hash_for_cache(input_file_path, input_dir)
        file_uuid_cached = is_file_cached(conn, fhash)
        if file_uuid_cached:
            logging.info(f"[SKIP cache-hit] {input_file_path}  already ingested as file_uuid={file_uuid_cached}")
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
        if isinstance(metadata, dict):
            sections = metadata.get("sections", []) or []
            comprehensive_questions = metadata.get("comprehensive_questions", []) or []
            # Combine sections and comprehensive_questions into a single structure
            if comprehensive_questions:
                sections_data = {
                    "sections": sections,
                    "comprehensive_questions": comprehensive_questions
                }
                sections = sections_data
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

        logging.info(f"[OK] {input_file_path} → {output_file_path} ({len(chunks)} chunks)")
        logging.info(f"Scheduled conversion for {input_file_path} to {output_file_path}")

    conn.close()
    content_logger.info(f"Completed content checking for directory: {input_dir}")
    logging.info(f"Completed processing for directory: {input_dir}")


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


def upsert_problem_meta(conn: sqlite3.Connection, file_uuid: str, pr: dict, q_id: int | str, q: dict):
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

def file_hash_for_cache(p: Path, root: Path) -> str:
    """ Generate a stable hash for a file, using its content if non-empty,
    or its relative path if empty.
    """
    try:
        if p.stat().st_size == 0:
            rel = p.relative_to(root).as_posix()
            return _blake2b_hex(f"EMPTY|{rel}")
    except Exception:
        return _blake2b_hex(f"EMPTY|{p.resolve().as_posix()}")

    return file_content_hash(p)


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

def gen_uuid() -> str:
    """
    Generate a new UUID.
    """
    return str(uuid.uuid4())

def is_file_cached(conn: sqlite3.Connection, file_hash: str) -> str | None:
    row = conn.execute(SQL_SELECT_FILE_UUID_BY_HASH, (file_hash,)).fetchone()
    return row[0] if row else None

def deterministic_chunk_uuid(file_uuid: str, idx: int, text: str = "") -> str:
    # stable across runs; include text if you want identity to change when content changes
    ns = uuid.UUID(file_uuid)
    name = f"{idx}:{hashlib.sha256(text.encode('utf-8')).hexdigest()[:16]}"
    return str(uuid.uuid5(ns, name))

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
  question_id, question, choices, answer, explanation
) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
ON CONFLICT(uuid) DO UPDATE SET
  file_uuid       = excluded.file_uuid,
  problem_index   = excluded.problem_index,
  problem_id      = excluded.problem_id,
  problem_content = excluded.problem_content,
  question_id     = excluded.question_id,
  question        = excluded.question,
  choices         = excluded.choices,
  answer          = excluded.answer,
  explanation     = excluded.explanation;
"""

SQL_SELECT_FILE_UUID_BY_HASH = "SELECT uuid FROM file WHERE file_hash=?;"


def update_problem_table_from_metadata_files(folder_path: Union[str, Path], chunk_db_path: Union[str, Path]) -> None:
    """
    Iterate through all .yaml files in a folder, extract uuid from metadata,
    check if it exists in the database, and add problems if the file is not already processed.

    Args:
        folder_path: Path to folder containing metadata.yaml files
        chunk_db_path: Path to the SQLite database
    """
    folder_path = Path(folder_path)
    if not folder_path.is_dir():
        raise ValueError(f"Provided folder path {folder_path} is not a directory.")

    conn = ensure_chunk_db(Path(chunk_db_path))

    try:
        # Find all .yaml files in the folder
        yaml_files = list(folder_path.rglob("*.yaml"))
        logging.info(f"Found {len(yaml_files)} yaml files to process")

        for yaml_file in yaml_files:
            try:
                # Load and parse the YAML file
                with yaml_file.open("r", encoding="utf-8") as f:
                    metadata = yaml.safe_load(f)

                if not isinstance(metadata, dict):
                    logging.warning(f"Skipping {yaml_file}: invalid metadata structure")
                    continue

                # Extract file information to generate file_uuid
                file_name = metadata.get("file_name")
                file_path = metadata.get("file_path")

                if not file_name or not file_path:
                    logging.warning(f"Skipping {yaml_file}: missing file_name or file_path")
                    continue

                # Try to find the actual file to generate hash and uuid
                # First try relative to yaml file location
                actual_file_path = yaml_file.parent / file_name
                if not actual_file_path.exists():
                    # Try using file_path from metadata
                    actual_file_path = yaml_file.parent / file_path
                    if not actual_file_path.exists():
                        logging.warning(f"Skipping {yaml_file}: cannot find actual file {file_name} or {file_path}")
                        continue

                # Generate file hash and uuid
                file_hash = file_hash_for_cache(actual_file_path, yaml_file.parent)
                file_uuid = deterministic_file_uuid(file_hash)

                # Check if file_uuid already exists in database
                existing_uuid = is_file_cached(conn, file_hash)
                if existing_uuid:
                    logging.info(f"Skipping {yaml_file}: file already exists in database with uuid {existing_uuid}")
                    continue

                # Extract problems from metadata
                problems = metadata.get("problems", [])
                if not problems:
                    logging.info(f"Skipping {yaml_file}: no problems found in metadata")
                    continue

                # First, ensure the file record exists in the database
                course_code = metadata.get("course_id", "")
                course_name = metadata.get("course_name", "")
                sections = metadata.get("sections", [])
                comprehensive_questions = metadata.get("comprehensive_questions", []) or []
                # Combine sections and comprehensive_questions into a single structure
                if comprehensive_questions:
                    sections_data = {
                        "sections": sections,
                        "comprehensive_questions": comprehensive_questions
                    }
                    sections = sections_data
                url = metadata.get("URL", "")

                upsert_file_meta(conn, {
                    "uuid": file_uuid,
                    "file_hash": file_hash,
                    "sections": sections,
                    "relative_path": str(actual_file_path.relative_to(yaml_file.parent)),
                    "course_code": course_code,
                    "course_name": course_name,
                    "file_name": file_name,
                    "extra_info": {},
                    "url": url,
                })

                # Insert problems into the database
                conn.execute("BEGIN IMMEDIATE")
                try:
                    for pr in problems:
                        q_container = pr.get("questions") or pr.get("question_set") or []
                        for q_id, q in _iter_questions_local(q_container):
                            upsert_problem_meta(conn, file_uuid, pr, q_id, q)
                    conn.commit()
                    logging.info(f"Successfully processed {yaml_file}: added {len(problems)} problems for file {file_name}")
                except Exception as e:
                    conn.rollback()
                    logging.error(f"Error inserting problems/comprehensive questions from {yaml_file}: {e}")
                    raise

            except yaml.YAMLError as e:
                logging.error(f"Error parsing YAML file {yaml_file}: {e}")
            except Exception as e:
                logging.error(f"Error processing {yaml_file}: {e}")

    finally:
        conn.close()

    logging.info("Completed updating problem table from metadata files")


def update_file_urls_from_metadata_files(folder_path: Union[str, Path], db_path: Union[str, Path]) -> None:
    """
    Iterate through all .yaml files in a folder, extract URL from metadata,
    and update the url column in the file table for existing files.

    Args:
        folder_path: Path to folder containing metadata.yaml files
        chunk_db_path: Path to the SQLite database
    """
    folder_path = Path(folder_path)
    if not folder_path.is_dir():
        raise ValueError(f"Provided folder path {folder_path} is not a directory.")

    conn = ensure_chunk_db(Path(db_path))

    try:
        # Find all .yaml files in the folder
        yaml_files = list(folder_path.rglob("*.yaml"))
        logging.info(f"Found {len(yaml_files)} yaml files to process for URL updates")

        updated_count = 0
        skipped_count = 0

        for yaml_file in yaml_files:
            try:
                # Load and parse the YAML file
                with yaml_file.open("r", encoding="utf-8") as f:
                    metadata = yaml.safe_load(f)

                if not isinstance(metadata, dict):
                    logging.warning(f"Skipping {yaml_file}: invalid metadata structure")
                    skipped_count += 1
                    continue

                # Extract URL from metadata
                url = metadata.get("URL")
                if not url:
                    logging.info(f"Skipping {yaml_file}: no URL found in metadata")
                    skipped_count += 1
                    continue

                # Extract file information to generate file_uuid
                file_name = metadata.get("file_name")
                file_path = metadata.get("file_path")

                if not file_name or not file_path:
                    logging.warning(f"Skipping {yaml_file}: missing file_name or file_path")
                    skipped_count += 1
                    continue

                # Try to find the actual file to generate hash and uuid
                # First try relative to yaml file location
                actual_file_path = yaml_file.parent / file_name
                if not actual_file_path.exists():
                    actual_file_path = yaml_file.parent / file_path
                    if not actual_file_path.exists():
                        logging.warning(f"Skipping {yaml_file}: cannot find actual file {file_name} or {file_path}")
                        skipped_count += 1
                        continue

                # Generate file hash and uuid
                file_hash = file_hash_for_cache(actual_file_path, yaml_file.parent)

                # Check if file exists in database
                existing_uuid = is_file_cached(conn, file_hash)
                if not existing_uuid:
                    logging.info(f"Skipping {yaml_file}: file not found in database")
                    skipped_count += 1
                    continue

                # Update the URL in the file table
                conn.execute("UPDATE file SET url = ? WHERE uuid = ?", (url, existing_uuid))
                conn.commit()
                
                updated_count += 1
                logging.info(f"Updated URL for file {file_name} (uuid: {existing_uuid}) to: {url}")

            except yaml.YAMLError as e:
                logging.error(f"Error parsing YAML file {yaml_file}: {e}")
                skipped_count += 1
            except Exception as e:
                logging.error(f"Error processing {yaml_file}: {e}")
                skipped_count += 1

        logging.info(f"Completed updating file URLs: {updated_count} updated, {skipped_count} skipped")

    finally:
        conn.close()

    logging.info("Completed updating file URLs from metadata files")

if __name__ == "__main__":
    update_file_urls_from_metadata_files(folder_path="/home/bot/bot/yk/YK_final/courses", db_path="/courses_out1/metadata.db")