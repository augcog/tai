# Standard python libraries
import json
import sqlite3
import threading
from typing import Optional
from pathlib import Path
from contextlib import contextmanager
from urllib.parse import quote

# Third-party libraries
import numpy as np
from openai import OpenAI

# Local libraries
from app.dependencies.model import get_embedding_engine
from app.config import settings

# Initialize embedding client (lazy loading)
_embedding_client: Optional[OpenAI] = None


def _get_embedding_client() -> OpenAI:
    """Get or initialize the embedding client."""
    global _embedding_client
    if _embedding_client is None:
        _embedding_client = get_embedding_engine()
    return _embedding_client


def _get_embedding(query: str) -> np.ndarray:
    """
    Get embedding for a query using vLLM embedding server via OpenAI API.
    Uses instruction-aware format for Qwen3-Embedding.
    """
    client = _get_embedding_client()
    # Qwen3-Embedding uses instruction-aware format
    formatted_query = f"Instruct: Given a web search query, retrieve relevant passages that answer the query\nQuery:{query}"

    response = client.embeddings.create(
        model=settings.vllm_embedding_model,
        input=formatted_query
    )
    return np.array(response.data[0].embedding, dtype=np.float32)


# Dynamic paths based on current file location
_CURRENT_FILE = Path(__file__).resolve()
_BACKEND_ROOT = _CURRENT_FILE.parent.parent.parent.parent  # Navigate up to ai_chatbot_backend/

# Environment Variables (using dynamic paths)
EMBEDDING_PICKLE_PATH = _BACKEND_ROOT / "app" / "embedding"
_DB_PATH = _BACKEND_ROOT / "db" / "metadata.db"
DB_URI_RO = f"file:{quote(str(_DB_PATH))}?mode=ro&cache=shared"
_local = threading.local()
# SQLDB: whether to use SQL database or Pickle for retrieval.
SQLDB = True


def _decode_vec_from_db(x) -> Optional[np.ndarray]:
    """
    Decode a vector from the database.
    """
    if x is None:
        return None
    if isinstance(x, (bytes, bytearray, memoryview)):
        return np.frombuffer(x, dtype=np.float32)
    elif isinstance(x, str):
        try:
            return np.asarray(json.loads(x), dtype=np.float32)
        except (json.JSONDecodeError, TypeError, ValueError):
            return None
    else:
        return None


def _init_conn(conn: sqlite3.Connection) -> None:
    """
    Initialize the SQLite connection with specific PRAGMAs for performance and consistency.
    """
    # Good perf/consistency tradeoff for read-heavy workloads
    conn.execute("PRAGMA synchronous=NORMAL;")
    conn.execute("PRAGMA temp_store=MEMORY;")
    # Wait a bit instead of throwing database is locked
    conn.execute("PRAGMA busy_timeout=3000;")
    conn.row_factory = sqlite3.Row
    conn.text_factory = str


def _get_conn() -> sqlite3.Connection:
    """
    Get a per-thread SQLite connection.
    """
    conn = getattr(_local, "conn", None)
    if conn is None:
        # Per-thread connection; do NOT share across threads
        conn = sqlite3.connect(DB_URI_RO, uri=True)  # keep check_same_thread=True (default)
        _init_conn(conn)
        _local.conn = conn
    return conn


@contextmanager
def _get_cursor():
    """
    Get a cursor from the SQLite connection.
    """
    cur = _get_conn().cursor()
    try:
        yield cur
    finally:
        cur.close()
