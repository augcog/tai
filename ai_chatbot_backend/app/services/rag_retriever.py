# Standard python libraries
import json
import os
import pickle
import sqlite3
import threading
import time
import urllib.parse
from typing import Any, Dict, List, Optional, Tuple
# Third-party libraries
import numpy as np
from pathlib import Path
from contextlib import contextmanager
from urllib.parse import quote
# Local libraries; import and initialize the embedding model from app dependencies.
from app.dependencies.model import get_embedding_engine
embedding_model = get_embedding_engine()
# Environment Variables
EMBEDDING_PICKLE_PATH = Path("/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/")
DB_URI_RO = f"file:{quote('/home/bot/localgpt/tai/ai_chatbot_backend/db/metadata.db')}?mode=ro&cache=shared"
_local = threading.local()
# SQLDB: whether to use SQL database or Pickle for retrieval.
SQLDB = True
_course_cache = {}
_course_lock = threading.Lock()


def _get_reference_documents(
    user_message: str, course: str, top_k: int
) -> Tuple[Tuple[List[str], List[str], List[str], List[float], List[str], List[str]], str]:
    """
    Retrieve top reference documents based on the query embedding and choice of DB-type.
    """
    picklefile, class_name = _get_pickle_and_class(course)
    t0 = time.time()
    query_embed = {"dense_vecs": embedding_model.encode(user_message, prompt_name="query")}
    print(f"[INFO] Embedding time: {time.time() - t0:.2f} seconds")
    t1 = time.time()
    if SQLDB:
        output = _get_references_from_sql(query_embed, course, top_k=top_k)
    else:
        output = _get_references_from_pickle(query_embed, picklefile, top_k=top_k)
    print(f"[INFO] Retrieval time: {time.time() - t1:.2f} seconds")
    return output, class_name


def _get_references_from_sql(
    query_embed: Dict[str, Any], course: str, top_k: int
) -> Tuple[List[str], List[str], List[str], List[float], List[str], List[str], List[str]]:
    """
    Retrieve top reference documents from a SQL database.
    """
    # Convert the query embedding to a numpy array
    qv = np.array(query_embed["dense_vecs"], dtype=np.float32).reshape(-1)
    if qv.size == 0:
        return [], [], [], [], [], [], []
    # Get the course index from the cache or build it if not present or stale
    idx = _get_course_index(course)
    if not idx.get("M") is not None:
        return [], [], [], [], [], [], []
    # Compute the scores for each document in the index
    M = idx["M"]
    scores = M @ qv
    k = min(top_k, M.shape[0])
    top_idx = np.argpartition(scores, -k)[-k:]
    top_idx = top_idx[np.argsort(scores[top_idx])[::-1]]
    # Extract the top-k results
    top_ids = [idx["ids"][i] for i in top_idx]
    top_docs = [idx["texts"][i] for i in top_idx]
    top_urls = [idx["urls"][i] for i in top_idx]
    top_scores = [float(scores[i]) for i in top_idx]
    top_files = [idx["files"][i] for i in top_idx]
    top_refs = [idx["refs"][i] for i in top_idx]
    top_titles = [idx["titles"][i] for i in top_idx]
    return top_ids, top_docs, top_urls, top_scores, top_files, top_refs, top_titles


def _get_course_index(course: str):
    """
    Get the course index from the cache or build it if not present or stale.
    """
    with _course_lock:
        idx = _course_cache.get(course)
    # check staleness
    with get_cursor() as cur:
        dv_now = cur.execute("PRAGMA data_version").fetchone()[0]
    if idx is None or idx["dv"] != dv_now or idx.get("M") is None:
        idx = _build_course_index(course)
        with _course_lock:
            _course_cache[course] = idx
    return idx


def _build_course_index(course: str):
    """
    Build an index of all chunks in the database for a specific course.
    """
    # Prepare connection params to the SQLite database
    where = "WHERE vector IS NOT NULL"
    params = []
    if course and course != "general":
        where += " AND course_id = ?"
        params.append(course)
    # Use a context manager to get SQL-DB cursor to ensure the connection is closed properly
    with get_cursor() as cur:
        dv = cur.execute("PRAGMA data_version").fetchone()[0]
        rows = cur.execute(f"""
            SELECT chunk_uuid, file_path, reference_path, vector, title, text, url
            FROM chunks
            {where};
        """, params).fetchall()
    # Process the rows fetched from the database
    ids, files, refs, titles, texts, urls, vecs = [], [], [], [], [], [], []
    dim = None
    for r in rows:
        v = _decode_vec_from_db(r["vector"])
        if v is None:
            continue
        if dim is None:
            dim = v.size
        if v.size != dim:
            continue
        ids.append(r["chunk_uuid"])
        files.append(r["file_path"] or "")
        refs.append(r["reference_path"] or "")
        titles.append(r["title"] or "")
        texts.append(r["text"] or "")
        urls.append(r["url"] or "")
        vecs.append(v.astype(np.float32, copy=False))
    if not vecs:
        return {"dv": dv, "M": None}
    # Stack the vectors to compute scores later
    M = np.vstack(vecs)  # shape: [N, D]
    return {
        "dv": dv, "ids": ids, "files": files, "refs": refs,
        "titles": titles, "texts": texts, "urls": urls, "M": M
    }


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
        except Exception:
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


def get_conn() -> sqlite3.Connection:
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
def get_cursor():
    """
    Get a cursor from the SQLite connection.
    """
    cur = get_conn().cursor()
    try:
        yield cur
    finally:
        cur.close()


def _get_references_from_pickle(
    query_embed: Dict[str, Any], picklefile: str, top_k: int
) -> Tuple[List[str], List[str], List[str], List[float],List[str],List[str]]:
    """
    Retrieve top reference documents from a pickle file.
    """
    # Load the pickle file content
    path_to_pickle = EMBEDDING_PICKLE_PATH / picklefile
    with open(path_to_pickle, "rb") as f:
        data_loaded = pickle.load(f)
    doc_list = data_loaded["doc_list"]
    file_path_list = data_loaded["file_paths_list"]
    topic_path_list = data_loaded["topic_path_list"]
    id_list = data_loaded["id_list"]
    url_list = data_loaded["url_list"]
    embedding_list = data_loaded["embedding_list"]
    # Compute the similarity scores
    score = query_embed["dense_vecs"] @ embedding_list["dense_vecs"].T
    score_array = np.array(score)
    # Sort the scores and retrieve the top_k results
    indices = np.argsort(score_array)[::-1]
    similarity_scores = np.sort(score_array)[-top_k:][::-1]
    top_ids = id_list[indices][:top_k]
    top_docs = doc_list[indices][:top_k]
    top_files = file_path_list[indices][:top_k]
    top_topic_paths = topic_path_list[indices][:top_k]
    top_urls = url_list[indices][:top_k].tolist()
    # TODO: Add top_titles if available in the pickle file
    top_titles = []
    return top_ids, top_docs, top_urls, similarity_scores, top_files, top_topic_paths, top_titles


def _get_pickle_and_class(course: str) -> Tuple[str, str]:
    """
    Return the pickle filename and course class name based on the course.
    """
    if course == "EECS 106B":
        return "eecs106b.pkl", "Robotic Manipulation and Interaction"
    elif course == "CS 61A":
        return "cs61a.pkl", "Structure and Interpretation of Computer Programs"
    elif course == "CS 294-137":
        return "cs294.pkl", "Immersive Computing and Virtual Reality"
    elif course == "Econ 140":
        return "econ140.pkl", "Econometrics"
    elif course == "INTD 315":
        return "language.pkl", "Multilingual Engagement"
    elif course == "ROAR Academy":
        return "ROAR Academy.pkl", "learning python and autonomous driving"
        # return "roar_academy.pkl", "learning python and autonomous driving"
    elif course == "general":
        return "Berkeley.pkl", "Berkeley"
    else:
        raise ValueError(f"Unknown course: {course}. Please provide a valid course name.")


#############################################################################
########################### DEVELOP-USEAGE ONLY #############################
#############################################################################
# TODO: REMOVE this code. Deprecated Soon.
def _sql_top_k_docs(
    query_embed: Dict[str, Any],
    course: Optional[str],
    k: int
) -> Tuple[List[str], List[str]]:
    """
    SQL 检索：返回
      - top_docs:        List[str]  选中的 chunk 文本
      - top_ref_paths:   List[str]  与之对应的 reference_path
    不归一化，直接用点积排序。
    """
    if k <= 0:
        return [], []

    qv = np.array(query_embed["dense_vecs"], dtype=np.float32).reshape(-1)
    if qv.size == 0:
        return [], []

    params: List[Any] = []
    where = "WHERE vector IS NOT NULL"
    if course and course != "general":
        where += " AND course_name = ?"
        params.append(course)
    with get_cursor() as cur:
        rows = cur.execute(
            f"""
            SELECT chunk_uuid, reference_path, vector
            FROM chunks
            {where};
            """,
            params
        ).fetchall()
    if not rows:
        return [], []

    ids: List[str] = []
    refs: List[str] = []
    vecs: List[np.ndarray] = []
    dim = None

    for chunk_uuid, reference_path, vec in rows:
        v = _decode_vec_from_db(vec)
        if v is None:
            continue
        if dim is None:
            dim = v.size
        if v.size != dim:
            continue
        ids.append(chunk_uuid)
        refs.append(reference_path or "")
        vecs.append(v.astype(np.float32, copy=False))

    if not vecs:
        return [], []

    M = np.vstack(vecs)         # (n, d)
    scores = M @ qv             # (n,)  直接点积

    k = min(k, len(ids))
    top_idx = np.argpartition(scores, -k)[-k:]
    top_idx = top_idx[np.argsort(scores[top_idx])[::-1]]

    top_ids  = [ids[i]  for i in top_idx]
    top_refs = [refs[i] for i in top_idx]

    placeholders = ",".join("?" for _ in top_ids)
    with get_cursor() as cur:
        detail_rows = cur.execute(
            f"SELECT chunk_uuid, text FROM chunks WHERE chunk_uuid IN ({placeholders});",
            top_ids
        ).fetchall()
    det = {cid: (txt or "") for cid, txt in detail_rows}

    top_docs = [det.get(cid, "") for cid in top_ids]
    return top_docs, top_refs


# TODO: REMOVE this code. Deprecated Soon.
def top_k_selector(
    message: str,
    stream: bool = True,
    rag: bool = True,
    course: Optional[str] = None,
    k: int = 3,
) -> Dict[str, Any]:
    if not rag or k <= 0:
        return {"top_docs": [], "top_reference_paths": [], "used_chunks": 0}

    t0 = time.time()
    query_embed = {"dense_vecs": embedding_model.encode(message, prompt_name="query")}
    print(f"Embedding time: {time.time() - t0:.2f} seconds")

    if SQLDB:
        top_docs, top_reference_paths = _sql_top_k_docs(query_embed, course, k)
        return {
            "top_docs": top_docs,
            "top_reference_paths": top_reference_paths,
            "used_chunks": min(len(top_docs), k),
        }

    current_dir = "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/"
    picklefile, _ = _get_pickle_and_class(course if course else "")
    path_to_pickle = os.path.join(current_dir, picklefile)
    with open(path_to_pickle, "rb") as f:
        data_loaded = pickle.load(f)
    doc_list = data_loaded["doc_list"]
    embedding_list = data_loaded["embedding_list"]
    score_simple = query_embed["dense_vecs"] @ embedding_list["dense_vecs"].T
    score_array = np.array(score_simple)
    indices = np.argsort(score_array)[::-1]
    top_indices = indices[:k]
    top_docs = [doc_list[i] for i in top_indices]
    return {
        "top_docs": top_docs,
        "top_reference_paths": [],
        "used_chunks": min(len(top_docs), k),
    }


#############################################################################
############################# LEGACY OLD CODE ###############################
#############################################################################
"""
LEGACY: made for endpoint testing; no longer in use.
"""
# TODO: Check for useage; remove if no use.
# def top_k_selector(
#     message: str,
#     stream: bool = True,
#     rag: bool = True,
#     course: Optional[str] = None,
#     k: int = 3,
# ) -> Dict[str, Any]:
#     """
#     Return the top-k documents relevant to the given message.
#     """
#     top_docs: List[str] = []
#     if rag:
#         picklefile, _ = _get_pickle_and_class(course if course else "")
#         current_dir = "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/"
#         # query_embed = embedding_model.encode_queries(
#         #     message, return_dense=True, return_sparse=True, return_colbert_vecs=True
#         # )
#         query_embed = {"dense_vecs": embedding_model.encode(message, prompt_name="query")}
#         if SQLDB:
#             embedding_db_path = os.path.join(current_dir, "embeddings.db")
#             db = sqlite3.connect(embedding_db_path)
#             db.enable_load_extension(True)
#             db.load_extension(EXT_VECTOR_PATH)
#             db.load_extension(EXT_VSS_PATH)
#             cur = db.cursor()
#             query_vector = query_embed["dense_vecs"].tolist()
#             query_vector_json = json.dumps(query_vector)
#             cur.execute(
#                 """
#                 SELECT rowid, distance
#                 FROM embeddings
#                 WHERE vss_search(embedding, ?)
#                 LIMIT ?;
#             """,
#                 (query_vector_json, k),
#             )
#             results = cur.fetchall()
#             db.commit()
#             db.close()
#             table_name = picklefile.replace(".pkl", "")
#             db_name = f"{table_name}.db"
#             main_db_path = os.path.join(current_dir, db_name)
#             db = sqlite3.connect(main_db_path)
#             cur = db.cursor()
#             indices = [result[0] for result in results]
#             placeholders = ",".join("?" for _ in indices)
#             query = f"SELECT doc_list FROM {table_name} WHERE rowid IN ({placeholders})"
#             cur.execute(query, indices)
#             results = cur.fetchall()
#             top_docs = [doc for (doc,) in results]
#             db.close()
#         else:
#             path_to_pickle = os.path.join(current_dir, picklefile)
#             with open(path_to_pickle, "rb") as f:
#                 data_loaded = pickle.load(f)
#             doc_list = data_loaded["doc_list"]
#             embedding_list = data_loaded["embedding_list"]
#             # combined_scores = bge_compute_score(
#             #     query_embed, embedding_list, [1, 1, 1], None, None
#             # )
#             # score_array = np.array(combined_scores["colbert+sparse+dense"])
#             score_simple = bge_compute_score(query_embed, embedding_list)
#             score_array = np.array(score_simple)
#             indices = np.argsort(score_array)[::-1]
#             top_indices = indices[:k]
#             top_docs = [doc_list[i] for i in top_indices]
#     used_chunks: int = min(len(top_docs), k)
#     return {"top_docs": top_docs, "used_chunks": used_chunks}
# def _get_reference_documents(
#     user_message: str, course: str, top_k: int
# ) -> Tuple[Tuple[List[str], List[str], List[str], List[float],List[str],List[str]], str]:
#     """
#     Retrieve top reference documents based on the query embedding.
#     """
#     picklefile, class_name = _get_pickle_and_class(course)
#     # Measure the time taken to compute the embedding
#     start_time = time.time()
#     query_embed = {"dense_vecs": embedding_model.encode(user_message, prompt_name="query")}
#     end_time = time.time()
#     print(f"Embedding time: {end_time - start_time:.2f} seconds")
#
#     if SQLDB:
#         return _get_references_from_sql(
#             query_embed, picklefile, top_k=top_k
#         ), class_name
#     else:
#         return _get_references_from_pickle(
#             query_embed, picklefile, top_k=top_k
#         ), class_name


"""
LEGACY: old sql db no longer used; to be refactored after Yikang finish conversion database.
"""
# TODO: Check for useage; remove if no use.
# TODO: Revise path design / configuration in the future for best practice.
# EXT_VECTOR_PATH: str = (
#     "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vector0"
# )
# EXT_VSS_PATH: str = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vss0"
# def clean_path(url_path: str) -> str:
#     """
#     Decode and clean a URL path.
#     """
#     decoded = urllib.parse.unquote(url_path)
#     cleaned = decoded.replace("%28", "(").replace("%29", ")").replace("%2B", "+")
#     cleaned = cleaned.replace(">", " > ").replace("(", " (").replace(")", ") ")
#     return " ".join(cleaned.split())
# def _get_references_from_sql(
#     query_embed: Dict[str, Any], picklefile: str, top_k: int
# ) -> Tuple[List[str], List[str], List[str], List[float],List[str],List[str]]:
#     """
#     Retrieve top reference documents from a SQL database.
#     """
#     current_dir = "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/"
#     embedding_db_path = os.path.join(current_dir, "embeddings.db")
#     db = sqlite3.connect(embedding_db_path)
#     db.enable_load_extension(True)
#     db.load_extension(EXT_VECTOR_PATH)
#     db.load_extension(EXT_VSS_PATH)
#     cur = db.cursor()
#     query_vector = query_embed["dense_vecs"].tolist()
#     query_vector_json = json.dumps(query_vector)
#     cur.execute(
#         """
#             SELECT rowid, distance
#             FROM embeddings
#             WHERE vss_search(embedding, ?)
#             LIMIT top_k;
#         """,
#         (query_vector_json,),
#     )
#     results = cur.fetchall()
#     db.commit()
#     db.close()
#
#     table_name = picklefile.replace(".pkl", "")
#     db_name = f"{table_name}.db"
#     main_db_path = os.path.join(current_dir, db_name)
#     db = sqlite3.connect(main_db_path)
#     cur = db.cursor()
#     indices = [result[0] for result in results]
#     similarity_scores = [result[1] for result in results]
#     placeholders = ",".join("?" for _ in indices)
#     query = f"SELECT id_list, doc_list, url_list FROM {table_name} WHERE rowid IN ({placeholders})"
#     cur.execute(query, indices)
#     results = cur.fetchall()
#     top_ids, top_docs, top_urls = [], [], []
#     for id_val, doc, url in results:
#         top_ids.append(id_val)
#         top_docs.append(doc)
#         top_urls.append(url)
#     db.close()
#     return top_ids, top_docs, top_urls, similarity_scores


"""
LEGACY: removed after migration into Qwen embedding models
"""
# TODO: Remove
# from FlagEmbedding import BGEM3FlagModel
# embedding_model = BGEM3FlagModel("BAAI/bge-m3", use_fp16=True,devices='cuda:0')
# query_embed = embedding_model.encode(
#     user_message, return_dense=True, return_sparse=True, return_colbert_vecs=True
# )
# def bge_compute_score(
#     query_embedding: Dict[str, Any],
#     document_embeddings: List[Dict[str, Any]],
#     weights_for_different_modes: Optional[List[float]] = None,
#     secondary_query_embedding: Optional[Dict[str, Any]] = None,
#     secondary_document_embeddings: Optional[List[Dict[str, Any]]] = None,
# ) -> Dict[str, List[float]]:
#     """
#     Compute combined similarity scores (dense, sparse, colbert, and weighted combinations).
#     Returns a dictionary of lists for each scoring mode.
#     """
#     scores: Dict[str, List[float]] = {
#         "colbert": [],
#         "sparse": [],
#         "dense": [],
#         "sparse+dense": [],
#         "colbert+sparse+dense": [],
#     }
#     if weights_for_different_modes is None:
#         weights_for_different_modes = [1, 1.0, 1.0]
#         weight_sum = 3
#     else:
#         assert len(weights_for_different_modes) == 3
#         weight_sum = sum(weights_for_different_modes)
#     scores["dense"] = query_embedding["dense_vecs"] @ document_embeddings["dense_vecs"].T
#     scores["sparse"] = embedding_model.compute_lexical_matching_score(
#         [query_embedding["lexical_weights"]], document_embeddings["lexical_weights"]
#     )[0]
#     for col in document_embeddings["colbert_vecs"]:
#         scores["colbert"].append(
#             embedding_model.colbert_score(query_embedding["colbert_vecs"], col)
#         )
#     scores["colbert+sparse+dense"]=(
#             scores["colbert"] * weights_for_different_modes[2]
#             + scores["sparse"] * weights_for_different_modes[1]
#             + scores["dense"] * weights_for_different_modes[0]
#         )/ weight_sum
#     return scores