# Standard python libraries
import os
import pickle
import time
from typing import Any, Dict, List, Optional, Tuple

# Third-party libraries
import numpy as np

# Local libraries
from app.services.query.embedding import (
    _get_embedding, _get_cursor, _decode_vec_from_db,
    SQLDB,
)


def _get_pickle_and_class(course: str) -> str:
    """
    Return the course class name based on the course.
    """
    if course == "EECS 106B":
        return "Robotic Manipulation and Interaction"
    elif course == "CS 61A":
        return "Structure and Interpretation of Computer Programs"
    elif course == "CS 294-137":
        return "Immersive Computing and Virtual Reality"
    elif course == "Econ 140":
        return "Econometrics"
    elif course == "INTD 315":
        return "Multilingual Engagement"
    elif course == "ROAR Academy":
        return  "learning python and autonomous driving"
    elif course == "Berkeley":
        return "Berkeley"
    elif course == "F1_racing":
        return "F1_racing"
    else:
        raise ValueError(f"Unknown course: {course}. Please provide a valid course name.")


#############################################################################
########################### DEPRECATED — DO NOT USE #########################
#############################################################################
# DEPRECATED: kept for backward compatibility with /top_k_docs endpoint.
# TODO: Remove once /top_k_docs endpoint is migrated or removed.
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
    query_embed = {"dense_vecs": _get_embedding(message)}
    print(f"Embedding time: {time.time() - t0:.2f} seconds")

    if SQLDB:
        top_docs, top_reference_paths = _sql_top_k_docs(query_embed, course, k)
        return {
            "top_docs": top_docs,
            "top_reference_paths": top_reference_paths,
            "used_chunks": min(len(top_docs), k),
        }

    current_dir = "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/"
    course = _get_pickle_and_class(course if course else "")
    path_to_pickle = os.path.join(current_dir, "picklefile")
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


# DEPRECATED: helper for top_k_selector
def _sql_top_k_docs(
    query_embed: Dict[str, Any],
    course: Optional[str],
    k: int
) -> Tuple[List[str], List[str]]:
    """
    SQL retrieval: returns
      - top_docs:        List[str]  selected chunk texts
      - top_ref_paths:   List[str]  corresponding reference_paths
    Uses dot product scoring without normalization.
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
    with _get_cursor() as cur:
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
    scores = M @ qv             # (n,)

    k = min(k, len(ids))
    top_idx = np.argpartition(scores, -k)[-k:]
    top_idx = top_idx[np.argsort(scores[top_idx])[::-1]]

    top_ids  = [ids[i]  for i in top_idx]
    top_refs = [refs[i] for i in top_idx]

    placeholders = ",".join("?" for _ in top_ids)
    with _get_cursor() as cur:
        detail_rows = cur.execute(
            f"SELECT chunk_uuid, text FROM chunks WHERE chunk_uuid IN ({placeholders});",
            top_ids
        ).fetchall()
    det = {cid: (txt or "") for cid, txt in detail_rows}

    top_docs = [det.get(cid, "") for cid in top_ids]
    return top_docs, top_refs
