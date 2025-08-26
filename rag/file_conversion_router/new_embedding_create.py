import sqlite3, json, numpy as np
from pathlib import Path
from typing import Optional, List

from sentence_transformers import SentenceTransformer

# ---------- tiny helpers ----------
def _title_path_from_json(title_json: Optional[str]) -> str:
    if not title_json:
        return ""
    try:
        arr = json.loads(title_json)
        if not isinstance(arr, (list, tuple)):
            arr = [str(arr)]
        parts = [str(x).strip() for x in arr if x is not None and str(x).strip()]
        return " > ".join(parts)
    except Exception:
        return str(title_json).strip()

def _to_blob(vec: np.ndarray) -> bytes:
    return np.asarray(vec, dtype=np.float32).tobytes()

def _ensure_vector_column(conn: sqlite3.Connection):
    # Add chunks.vector if missing
    cols = {row[1] for row in conn.execute("PRAGMA table_info(chunks)")}
    if "vector" not in cols:
        conn.execute("ALTER TABLE chunks ADD COLUMN vector BLOB")

# ---------- MVP: write embeddings into chunks.vector ----------
def embedding_create(db_path, embedding_name=None, ):
    """
    Compute embeddings and save into chunks.vector (BLOB, float32 bytes).

    Args:
      db_path: path to SQLite DB.
      embedding_name: optional course filter; matches course_name OR course_id.
      only_missing: if True, only embed rows where vector is NULL/empty.
    """
    db_path = Path(db_path)
    if not db_path.exists():
        raise FileNotFoundError(f"DB not found: {db_path}")

    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row

    # 0) Ensure schema has 'vector' column
    _ensure_vector_column(conn)

    # 1) Select target chunks
    params: List[str] = []
    where = []
    if embedding_name and str(embedding_name).strip():
        where.append("(course_name = ? OR course_code = ?)")
        params.extend([embedding_name, embedding_name])
    sql = """
    SELECT
      chunk_uuid, text, title, reference_path, file_path, course_name, course_code, file_uuid, idx
    FROM chunks
    """
    if where:
        sql += " WHERE " + " AND ".join(where)
    sql += " ORDER BY file_uuid, idx, chunk_uuid"

    rows = conn.execute(sql, params).fetchall()
    if not rows:
        conn.close()
        print("[embed] no chunks matched the current filter")
        return

    chunk_uuids  = [r["chunk_uuid"] for r in rows]
    texts        = [r["text"] or "" for r in rows]
    titles_json  = [r["title"] for r in rows]
    title_paths  = [_title_path_from_json(tj) for tj in titles_json]

    # 2) Build prompts
    hp_list = [
        f"document_hierarchy_path: {tp}\ndocument: {txt}\n"
        for tp, txt in zip(title_paths, texts)
    ]

    # 3) Encode
    model = SentenceTransformer(
        "Qwen/Qwen3-Embedding-4B",
        model_kwargs={
            "attn_implementation": "flash_attention_2",
            "torch_dtype": "auto",
            "device_map": "auto",
        },
        tokenizer_kwargs={"padding_side": "left"},
    )
    dense = np.asarray(model.encode(hp_list))
    if dense.ndim != 2 or dense.shape[0] != len(chunk_uuids):
        conn.close()
        raise ValueError(f"Embeddings shape mismatch: {dense.shape} vs {len(chunk_uuids)} chunks")

    # 4) Write back into chunks.vector (transactional)
    try:
        conn.execute("BEGIN IMMEDIATE")
        cur = conn.cursor()
        cur.executemany(
            "UPDATE chunks SET vector=? WHERE chunk_uuid=?",
            [( _to_blob(dense[i]), chunk_uuids[i]) for i in range(len(chunk_uuids))]
        )
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()

    print(f"[embed] wrote {len(chunk_uuids)} vectors into chunks.vector @ {db_path}")

if __name__ == "__main__":
    embedding_create(
        "/home/bot/bot/yk/YK_final/courses_out/course.db",
        "ROAR Academy",
    )
