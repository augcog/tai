"""
ingest.py  —  YAML ➜ flattened SQLite
-------------------------------------
* One DB (content.db)
* Tables: file, problem
* Each question becomes one row in problem
"""
import json, uuid, yaml, sqlite3, pathlib
from typing import List, Dict, Any
import os

# ───────────────────────── helpers ────────────────────────────────────────────
ROOT = pathlib.Path("/home/bot/bot/yk/YK_final/courses")
DB_PATH = ROOT / "metadata.db"


def gen_uuid() -> str:
    return str(uuid.uuid4())


def jdump(obj, default=None) -> str:  # compact helper
    if default is None:
        default = []
    return json.dumps(obj, ensure_ascii=False, default=default)


def load_yaml_dir(dir_: str | pathlib.Path) -> List[Dict]:
    """
    Recursively read every .yml/.yaml file, return a single list[dict].
    """
    out: List[Dict] = []
    for fp in pathlib.Path(dir_).rglob("*.yml"):
        out.extend(_load(fp))
    for fp in pathlib.Path(dir_).rglob("*.yaml"):
        out.extend(_load(fp))
    return out


def _load(path: pathlib.Path) -> List[Dict]:
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    return doc if isinstance(doc, list) else [doc]


# ──────────────────── DB bootstrap (one file, two tables) ─────────────────────
if DB_PATH.exists():              # ❶   check for an old file
    DB_PATH.unlink()              # ❷   remove it
if not DB_PATH.parent.exists():
    os.makedirs(DB_PATH.parent)

# now create a brand-new, empty DB on next connect

db = sqlite3.connect(DB_PATH)
with db:
    db.executescript(
        """
        /* file-level metadata ---------------------------------------------- */
        CREATE TABLE IF NOT EXISTS file (
            uuid       TEXT PRIMARY KEY,
            file_name  TEXT NOT NULL,
            url        TEXT,
            sections   TEXT,              -- JSON blob
            relative_path TEXT DEFAULT '' UNIQUE NOT NULL, -- relative path to the file in the course directory
            course_code TEXT DEFAULT '', -- course code, e.g. "CS61A"
            course_name TEXT DEFAULT ''  -- course name, e.g. "CS61A: Structure and Interpretation of Computer Programs"
        );

        /* one row per question --------------------------------------------- */
        CREATE TABLE IF NOT EXISTS problem (
            uuid            TEXT PRIMARY KEY,
            file_uuid       TEXT,         -- FK → file(uuid)
            problem_index   REAL,
            problem_id      TEXT,
            problem_content TEXT,         -- optional raw stem/context
            question_id     INT,
            question        TEXT,
            choices         TEXT,         -- JSON list[str]
            answer          TEXT,         -- JSON list[int]
            explanation     TEXT
        );
        """
    )

# ─────────────────────────── ingestion logic ─────────────────────────────────
def ingest(files: List[Dict[str, Any]]) -> None:
    """
    files = list of top-level dicts (one per YAML 'file')
    Each dict may have 0..n 'problems'
    """
    for f in files:
        if not f.get("file_name") or not(f.get('file_path').startswith(f.get('course_id')) or f.get('file_path').startswith('CS 61A')) or 'course_website' in f.get('file_path'):
            continue
        file_uuid = gen_uuid()
        section=f.get("sections")
        if section:
            index_list= [x['index'] for x in section]
            # check if index order is correct
            if index_list != sorted(index_list):
                print(f"❌ Skipping {f['file_path']} due to incorrect section index order: {index_list}")
                continue
        db.execute(
            """
            INSERT INTO file (uuid, file_name, url, sections, relative_path, course_code, course_name)
            VALUES (?, ?, ?, ?, ?, ?,?)
            """,
            (
                file_uuid,
                f["file_name"],
                f.get("URL"),
                jdump(f.get("sections")),
                f['file_path'],
                f['course_id'],
                f['course_name'],
            ),
        )

        for pr in f.get("problems", {}):  # optional
            p_index = pr.get("problem_index")
            p_content = pr.get("problem_content")  # may be None
            p_id= pr.get("problem_id")  # e.g. "1.2.3" or "
            question_id=[1,2]
            # flatten every question under this problem ----------------------
            for id in question_id:
                q = pr["questions"][f'question_{id}']
                question_uuid = gen_uuid()

                db.execute(
                    """
                    INSERT INTO problem
                    (uuid, file_uuid, problem_index, problem_id,
                     problem_content,question_id, question, choices, answer, explanation)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?,?)
                    """,
                    (
                        question_uuid,
                        file_uuid,
                        p_index,
                        p_id,
                        p_content,
                        id,  # question_id
                        q.get("question"),
                        jdump(q.get("choices")),
                        jdump(q.get("answer")),  # list[int]
                        q.get("explanation"),
                    ),
                )

    db.commit()


# ───────────────────────────── CLI entrypoint ────────────────────────────────
if __name__ == "__main__":
    payload = load_yaml_dir(ROOT)
    ingest(payload)
    print(f"✅ Imported {len(payload)} files and their questions into {DB_PATH}")
