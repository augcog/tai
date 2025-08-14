"""Internal service to process a directory of files and schedule conversion tasks for each file."""

import logging
import uuid
import hashlib
# from concurrent.futures import as_completed
from pathlib import Path
from typing import Dict, Type, Union
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
    ".json": EdConverter,
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
        course_id: str,
        log_dir: Union[str, Path] = None,
        chunk_db_path: Union[str, Path] = None,
        metadata_db_path: Union[str, Path] = None,
) -> None:
    """Walk through the input directory and schedule conversion tasks for specified file types."""
    logging.getLogger().setLevel(logging.INFO)
    output_dir = Path(output_dir)
    input_dir = Path(input_dir)
    # open content DB (files/chunks)
    conn = ensure_chunk_db(Path(chunk_db_path))
    # open metadata DB (file/problem)
    conn_meta: sqlite3.Connection | None = None
    if metadata_db_path:
        p = Path(metadata_db_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        conn_meta = sqlite3.connect(str(p))
        conn_meta.row_factory = sqlite3.Row
        ensure_metadata_db(conn_meta)

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
        """Check if the file path should be ignored based on the patterns."""
        relative_path = str(file_path.relative_to(input_dir))
        should_ignore = spec.match_file(relative_path)
        if should_ignore:
            logging.info(f"Ignoring file: {relative_path} based on ignore patterns.")
        return should_ignore

    # Iterate over all files with specified extensions
    for input_file_path in input_dir.rglob("*"):
        if not (input_file_path.is_file() and input_file_path.suffix in valid_extensions):
            continue
        if should_ignore(input_file_path):
            continue
        fhash =  file_hash_for_cache(input_file_path, input_dir)
        file_uuid = is_file_cached(conn, fhash)
        if file_uuid:
            logging.info(
                f"[SKIP cache-hit] {input_file_path}  already ingested as file_uuid={file_uuid}")
            continue

        # Construct the output subdirectory and file path
        output_subdir = output_dir / input_file_path.relative_to(input_dir).parent
        output_subdir.mkdir(parents=True, exist_ok=True)
        output_file_path = output_subdir / input_file_path.stem

        # Instantiate a new converter object for each file based on the file extension
        converter_class = converter_mapping.get(input_file_path.suffix)
        if converter_class:
            file_uuid = deterministic_file_uuid(fhash)
            converter = converter_class(course_name, course_id, file_uuid)
            (chunks, metadata), _ = converter.convert(input_file_path, output_file_path, input_dir.parent)
            if not chunks:
                chunks = []
            fp = chunks[0].file_path if chunks else input_file_path.relative_to(input_dir)
            relative_path = str(fp)

            write_chunks_to_db(
                conn=conn,
                file_hash=fhash,
                chunks=chunks,
                relative_path=relative_path,
                file_name=input_file_path.name,
            )

            # unpack metadata fields
            url = ""
            sections = []
            if isinstance(metadata, dict):
                url = metadata.get("URL")
                sections = metadata.get("sections", [])
            file_row = {
                "file_uuid": file_uuid,
                "file_name": input_file_path.name,
                "URL": url,
                "sections": sections,
                "file_path": relative_path,
                "course_id": course_id,
                "course_name": course_name,
            }

            conn_meta.execute("BEGIN IMMEDIATE")
            upsert_file_meta(conn_meta, file_row)

            # upsert problems if present
            problems = []
            if isinstance(metadata, dict):
                problems = metadata.get("problems") or metadata.get("problem_set") or []
            for pr in problems:
                q_container = pr.get("questions") or pr.get("question_set") or []
                for q_id, q in _iter_questions_local(q_container):
                    upsert_problem_meta(conn_meta, file_uuid, pr, q_id, q)

            conn_meta.commit()

            logging.info(f"[OK] {input_file_path} → {output_file_path} ({len(chunks)} chunks)")
            # future = schedule_conversion(
            #     converter.convert, input_file_path, output_file_path, input_dir
            # )
            # futures.append(future)
            logging.info(
                f"Scheduled conversion for {input_file_path} to {output_file_path}"
            )
        else:
            logging.warning(
                f"No converter available for file type {input_file_path.suffix}"
            )

    # for future in as_completed(futures):
    #     # try:
    #     result = future.result()
    #     logging.info(f"Conversion result: {result}")
    #     # Handle the successful result here
    #     logging.info("Task completed successfully.")
    #     # except Exception as e:
    #     #     logging.error(f"Conversion failed: {e}")
    conn.close()
    if conn_meta:
        conn_meta.close()
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
    conn.executescript(SQL_INIT)
    return conn

def ensure_metadata_db(conn_meta: sqlite3.Connection):
    conn_meta.executescript(SQL_INIT_METADATA)

def upsert_file_meta(conn_meta: sqlite3.Connection, f: dict):
    """
    Upsert into `file` using ONLY uuid as the conflict key.
    """
    args = (
        f.get("file_uuid"),                       # uuid (PRIMARY KEY)
        f["file_name"],
        f.get("URL",""),
        json.dumps(f.get("sections",[]), ensure_ascii=False),
        f.get("file_path", ""),                                 # stored but NOT used for dedupe
        f.get("course_id", ""),                                 # -> course_code
        f.get("course_name", ""),
    )
    conn_meta.execute(SQL_UPSERT_FILE_BY_UUID, args)


def upsert_problem_meta(conn_meta: sqlite3.Connection, file_uuid: str, pr: dict, q_id: int, q: dict):
    conn_meta.execute(
        SQL_UPSERT_PROBLEM,
        (
            str(uuid.uuid4()),                # problem.uuid (row PK)
            file_uuid,                        # FK → file.uuid
            pr["problem_index"],
            pr["problem_id"],
            pr["problem_content"],
            q_id,
            q["question"],
            json.dumps(q["choices"], ensure_ascii=False),
            json.dumps(q["answer"], ensure_ascii=False),
            q["explanation"],
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

def _title_to_text(titles) -> str | None:
    if titles is None:
        return None
    if isinstance(titles, str):
        return titles
    return json.dumps(titles, ensure_ascii=False)

def _to_json_or_none(obj):
    return json.dumps(obj, ensure_ascii=False) if obj is not None else None

def write_chunks_to_db(conn: sqlite3.Connection,
                       file_hash: str,
                        relative_path: str,
                        file_name: str,
                       chunks: List[Chunk]) -> None:
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


    conn.execute("BEGIN IMMEDIATE")
    # 2) Bind file_hash ↔ file_uuid. If hash exists, must match given file_uuid.
    row = conn.execute(SQL_SELECT_FILE_UUID_BY_HASH, (file_hash,)).fetchone()
    if row:
        existing_uuid = row[0]
        if existing_uuid != file_uuid:
            raise ValueError(
                f"file_hash {file_hash} is already bound to file_uuid {existing_uuid}, "
                f"but chunks carry file_uuid {file_uuid}"
            )
        # refresh file_name/relative_path if we have values
        if file_name or relative_path:
            conn.execute(SQL_UPDATE_FILE_META, (file_name, relative_path, file_hash))
    else:
        conn.execute(SQL_INSERT_FILE, (file_uuid, file_hash, file_name, relative_path))

    # Upsert chunks
    for i, ch in enumerate(chunks):
        idx = ch.index
        text = ch.content
        if not isinstance(text, str):
            raise ValueError(f"chunk[{i}] content must be str")

        title_list = _title_to_text(ch.titles)
        title_json = _to_json_or_none(title_list)
        url_json = _to_json_or_none(ch.chunk_url)
        fp = str(ch.file_path)
        ref_path = str(getattr(ch, "reference_path", None))
        course_name = getattr(ch, "course_name", None)
        course_id = getattr(ch, "course_id", None)
        chunk_index = getattr(ch, "chunk_index", None)

        conn.execute(
            SQL_UPSERT_CHUNK,
            (ch.chunk_uuid, file_uuid, idx, text, title_json, url_json, fp, ref_path, course_name, course_id, chunk_index),
        )

    conn.commit()

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
    """
    Return existing file_uuid if file_hash is already present; else None.
    """
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


# ───────────────────────── content DB (files/chunks) ─────────────────────────
SQL_INIT = """
PRAGMA foreign_keys=ON;
PRAGMA journal_mode=WAL;
PRAGMA synchronous=NORMAL;

CREATE TABLE IF NOT EXISTS files (
  file_uuid     TEXT PRIMARY KEY,
  file_hash     TEXT NOT NULL UNIQUE,
  file_name     TEXT,
  relative_path TEXT
);

CREATE TABLE IF NOT EXISTS chunks (
  chunk_uuid     TEXT PRIMARY KEY,
  file_uuid      TEXT NOT NULL,
  idx            INTEGER NOT NULL,
  text           TEXT NOT NULL,
  title          TEXT,   -- JSON string
  url            TEXT,   -- JSON string
  file_path      TEXT,
  reference_path TEXT,
  course_name    TEXT,
  course_id      TEXT,
  chunk_index    INTEGER, 
  FOREIGN KEY (file_uuid) REFERENCES files(file_uuid) ON DELETE CASCADE,
  UNIQUE(file_uuid, chunk_index)
);

CREATE INDEX IF NOT EXISTS idx_files_hash  ON files(file_hash);
CREATE INDEX IF NOT EXISTS idx_chunks_file ON chunks(file_uuid);
CREATE INDEX IF NOT EXISTS idx_chunks_ttl  ON chunks(title);
"""

SQL_SELECT_FILE_UUID_BY_HASH = "SELECT file_uuid FROM files WHERE file_hash=?;"

SQL_INSERT_FILE = """
INSERT INTO files(file_uuid, file_hash, file_name, relative_path)
VALUES(?, ?, ?, ?);
"""

SQL_UPDATE_FILE_META = """
UPDATE files
SET file_name = ?, relative_path = ?
WHERE file_hash = ?;
"""

SQL_UPSERT_CHUNK = """
INSERT INTO chunks(
  chunk_uuid, file_uuid, idx, text, title, url, file_path, reference_path, course_name, course_id, chunk_index
) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
ON CONFLICT(chunk_uuid) DO UPDATE SET
  file_uuid      = excluded.file_uuid,
  idx            = excluded.idx,
  text           = excluded.text,
  title          = excluded.title,
  url            = excluded.url,
  file_path      = excluded.file_path,
  reference_path = excluded.reference_path,
  course_name    = excluded.course_name,
  course_id      = excluded.course_id,
  chunk_index    = excluded.chunk_index;
"""


# ───────────────────────── metadata DB (file/problem) ────────────────────────
SQL_INIT_METADATA = """
PRAGMA foreign_keys=ON;
PRAGMA journal_mode=WAL;
PRAGMA synchronous=NORMAL;

CREATE TABLE IF NOT EXISTS file (
  uuid         TEXT PRIMARY KEY,
  file_name    TEXT,
  URL          TEXT,
  sections     TEXT,    -- JSON string
  file_path    TEXT,
  course_id    TEXT,
  course_name  TEXT
);

CREATE TABLE IF NOT EXISTS problem (
  uuid            TEXT PRIMARY KEY,
  file_uuid       TEXT NOT NULL,
  problem_index   INTEGER,
  problem_id      TEXT,
  problem_content TEXT,
  question_id     TEXT,
  question        TEXT,
  choices         TEXT,   -- JSON string
  answer          TEXT,   -- JSON string
  explanation     TEXT,
  FOREIGN KEY (file_uuid) REFERENCES file(uuid) ON DELETE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_problem_file ON problem(file_uuid);
"""

SQL_UPSERT_FILE_BY_UUID = """
INSERT INTO file (uuid, file_name, URL, sections, file_path, course_id, course_name)
VALUES (?, ?, ?, ?, ?, ?, ?)
ON CONFLICT(uuid) DO UPDATE SET
  file_name   = excluded.file_name,
  URL         = excluded.URL,
  sections    = excluded.sections,
  file_path   = excluded.file_path,
  course_id   = excluded.course_id,
  course_name = excluded.course_name;
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
