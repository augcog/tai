import sqlite3, json, numpy as np, logging
from pathlib import Path
from typing import Optional, List

from sentence_transformers import SentenceTransformer

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

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
        logging.info("Adding 'vector' column to chunks table")
        conn.execute("ALTER TABLE chunks ADD COLUMN vector BLOB")
        logging.info("✓ Vector column added successfully")
    else:
        logging.debug("Vector column already exists")

def embedding_create(db_path, embedding_name=None, ):
    """
    Compute embeddings and save into chunks.vector (BLOB, float32 bytes).

    Args:
      db_path: path to SQLite DB.
      embedding_name: optional course filter; matches course_name OR course_id.
      only_missing: if True, only embed rows where vector is NULL/empty.
    """
    db_path = Path(db_path)
    logging.info(f"Starting embedding process for database: {db_path}")

    if embedding_name:
        logging.info(f"Filtering by course: {embedding_name}")
    else:
        logging.info("Processing all courses")

    if not db_path.exists():
        logging.error(f"Database not found: {db_path}")
        raise FileNotFoundError(f"DB not found: {db_path}")

    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row

    # 0) Ensure schema has 'vector' column
    _ensure_vector_column(conn)

    # 1) Select target chunks (only those without embeddings)
    params: List[str] = []
    where = ["(vector IS NULL OR vector = '')"]
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
        logging.info("✓ No chunks found that need embedding - all chunks are up to date")
        return

    logging.info(f"Found {len(rows)} chunks that need embedding")
    logging.info(f"Database: {db_path.name}")

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
    logging.info("Loading embedding model: Qwen/Qwen3-Embedding-4B...")
    model = SentenceTransformer(
        "Qwen/Qwen3-Embedding-4B",
        model_kwargs={
            "attn_implementation": "flash_attention_2",
            "torch_dtype": "auto",
            "device_map": "auto",
        },
        tokenizer_kwargs={"padding_side": "left"},
    )
    logging.info("Model loaded successfully")

    logging.info(f"Encoding {len(hp_list)} chunks...")
    dense = np.asarray(model.encode(hp_list))
    logging.info(f"Encoding complete. Generated embeddings shape: {dense.shape}")

    if dense.ndim != 2 or dense.shape[0] != len(chunk_uuids):
        conn.close()
        logging.error(f"Embeddings shape mismatch: {dense.shape} vs {len(chunk_uuids)} chunks")
        raise ValueError(f"Embeddings shape mismatch: {dense.shape} vs {len(chunk_uuids)} chunks")

    try:
        logging.info(f"Writing {len(chunk_uuids)} embeddings to database...")
        conn.execute("BEGIN IMMEDIATE")
        cur = conn.cursor()
        cur.executemany(
            "UPDATE chunks SET vector=? WHERE chunk_uuid=?",
            [( _to_blob(dense[i]), chunk_uuids[i]) for i in range(len(chunk_uuids))]
        )
        conn.commit()
        logging.info(f"✓ Successfully wrote {len(chunk_uuids)} vectors to chunks.vector")
        logging.info(f"✓ Embedding process completed for {db_path.name}")
    except Exception as e:
        conn.rollback()
        logging.error(f"Failed to write embeddings to database: {e}")
        raise
    finally:
        conn.close()

if __name__ == "__main__":
    embedding_create(
        "/home/bot/bot/yk/YK_final/courses_out/db/CS 61A_metadata.db",
        "CS 61A",
    )
    embedding_create(
        "/home/bot/bot/yk/YK_final/courses_out/db/ROAR Academy_metadata.db",
        "ROAR Academy",
    )
    embedding_create(
        "/home/bot/bot/yk/YK_final/courses_out/db/Berkeley_metadata.db",
        "Berkeley",
    )
