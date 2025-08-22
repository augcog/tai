import sqlite3, json, numpy as np
from pathlib import Path
from typing import Optional, List

from FlagEmbedding import BGEM3FlagModel
from sentence_transformers import SentenceTransformer

# ---------------- schema helpers ----------------
def _ensure_embeddings_table(conn: sqlite3.Connection):
    """
    Create the new embeddings table if it doesn't exist.
    NOTE: If you already had an older 'embeddings' table with NOT NULL model/dim,
    drop/migrate it first or rename it, otherwise inserts may fail.
    """
    conn.executescript("""
    PRAGMA foreign_keys=ON;

    CREATE TABLE IF NOT EXISTS embeddings(
      chunk_uuid     TEXT PRIMARY KEY,
      vector         BLOB NOT NULL,      -- np.float32 bytes
      course_name    TEXT,
      course_id      TEXT,
      title          TEXT,               -- JSON string (copied from chunks.title)
      reference_path TEXT,
      file_path      TEXT,
      FOREIGN KEY(chunk_uuid) REFERENCES chunks(chunk_uuid) ON DELETE CASCADE
    );

    CREATE INDEX IF NOT EXISTS idx_emb_course_name ON embeddings(course_name);
    CREATE INDEX IF NOT EXISTS idx_emb_course_id   ON embeddings(course_id);
    """)

def _title_path_from_json(title_json: Optional[str]) -> str:
    if not title_json:
        return ""
    try:
        arr = json.loads(title_json)
        if not isinstance(arr, (list, tuple)):
            arr = [str(arr)]
        parts = [str(x).strip() for x in arr if x is not None and str(x).strip()]
        # titles are big -> small; join as a readable path
        return " > ".join(parts)
    except Exception:
        return str(title_json).strip()

def _to_blob(vec: np.ndarray) -> bytes:
    return np.asarray(vec, dtype=np.float32).tobytes()

# ---------------- MVP embedding function ----------------
def embedding_create(db_path,embedding_name, model):
    """
    MVP replacement based on your updated DB schema.

    Args (kept for compatibility with your caller):
      - db_path: path to the SQLite DB that contains 'files' and 'chunks'
      - name: unused (kept for signature compatibility)
      - embedding_name: optional course filter; matches course_name OR course_id
      - folder_name: unused (kept for signature compatibility)
      - model: "BGE" or "Qwen"
    """
    db_path = Path(db_path)
    if not db_path.exists():
        raise FileNotFoundError(f"DB not found: {db_path}")

    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    _ensure_embeddings_table(conn)

    # 1) Load chunks (optionally filter by course name/id via `embedding_name`)
    params: List[str] = []
    sql = """
    SELECT
      chunk_uuid, text, title, reference_path, file_path, course_name, course_id, file_uuid, idx
    FROM chunks
    """
    if embedding_name and str(embedding_name).strip():
        sql += " WHERE course_name = ? OR course_id = ?"
        params.extend([embedding_name, embedding_name])
    sql += " ORDER BY file_uuid, idx, chunk_uuid"

    rows = conn.execute(sql, params).fetchall()
    if not rows:
        conn.close()
        print("[embed] no chunks matched the current filter")
        return

    chunk_uuids   = [r["chunk_uuid"] for r in rows]
    texts         = [r["text"] or "" for r in rows]
    titles_json   = [r["title"] for r in rows]
    ref_paths     = [r["reference_path"] for r in rows]
    file_paths    = [r["file_path"] for r in rows]
    course_names  = [r["course_name"] for r in rows]
    course_ids    = [r["course_id"] for r in rows]

    title_paths = [_title_path_from_json(tj) for tj in titles_json]

    # 2) Build prompts (same style you used earlier)
    hp_list = [
        f"document_hierarchy_path: {tp}\ndocument: {txt}\n"
        for tp, txt in zip(title_paths, texts)
    ]

    embedding_model = SentenceTransformer(
        "Qwen/Qwen3-Embedding-4B",
        model_kwargs={
            "attn_implementation": "flash_attention_2",
            "torch_dtype": "auto",
            "device_map": "auto",
        },
        tokenizer_kwargs={"padding_side": "left"},
    )
    dense = np.asarray(embedding_model.encode(hp_list), dtype=np.float32)
    if dense.ndim != 2 or dense.shape[0] != len(chunk_uuids):
        conn.close()
        raise ValueError(f"Embeddings shape mismatch: {dense.shape} vs {len(chunk_uuids)} chunks")

    # 4) Write back into embeddings (UPSERT by chunk_uuid)
    SQL_UPSERT = """
    INSERT INTO embeddings(chunk_uuid, vector, course_name, course_id, title, reference_path, file_path)
    VALUES(?, ?, ?, ?, ?, ?, ?)
    ON CONFLICT(chunk_uuid) DO UPDATE SET
      vector         = excluded.vector,
      course_name    = excluded.course_name,
      course_id      = excluded.course_id,
      title          = excluded.title,
      reference_path = excluded.reference_path,
      file_path      = excluded.file_path;
    """

    try:
        conn.execute("BEGIN IMMEDIATE")
        cur = conn.cursor()
        for i, cu in enumerate(chunk_uuids):
            cur.execute(
                SQL_UPSERT,
                (
                    cu,
                    _to_blob(dense[i]),
                    course_names[i],
                    course_ids[i],
                    titles_json[i],
                    ref_paths[i],
                    file_paths[i],
                ),
            )
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()

    print(f"[embed] wrote {len(chunk_uuids)} embeddings into {db_path}")



if __name__ == "__main__":
    embedding_create("/courses_out1/ROAR Academy_output/ROAR Academy_chunk.db",
                     "ROAR Academy",
                     "QWEN")