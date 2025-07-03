import json
import os
import pickle
import sqlite3
import urllib.parse
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from FlagEmbedding import BGEM3FlagModel

# Keep your embedding_model or pass it in, as suits your design
embedding_model = BGEM3FlagModel("BAAI/bge-m3", use_fp16=True)

# Global configuration for SQL database usage and extension paths.
SQLDB: bool = False
# TODO: Revise path design / configuration in the future for best practice.
EXT_VECTOR_PATH: str = (
    "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vector0"
)
EXT_VSS_PATH: str = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vss0"


def bge_compute_score(
    query_embedding: Dict[str, Any],
    document_embeddings: List[Dict[str, Any]],
    weights_for_different_modes: Optional[List[float]] = None,
    secondary_query_embedding: Optional[Dict[str, Any]] = None,
    secondary_document_embeddings: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, List[float]]:
    """
    Compute combined similarity scores (dense, sparse, colbert, and weighted combinations).
    Returns a dictionary of lists for each scoring mode.
    """
    scores: Dict[str, List[float]] = {
        "colbert": [],
        "sparse": [],
        "dense": [],
        "sparse+dense": [],
        "colbert+sparse+dense": [],
    }
    if weights_for_different_modes is None:
        weights_for_different_modes = [1, 1.0, 1.0]
        weight_sum = 3
        print("Default weights for dense, sparse, colbert are [1.0, 1.0, 1.0]")
    else:
        assert len(weights_for_different_modes) == 3
        weight_sum = sum(weights_for_different_modes)

    for doc_embed in document_embeddings:
        dense_score = query_embedding["dense_vecs"] @ doc_embed["dense_vecs"].T
        sparse_score = embedding_model.compute_lexical_matching_score(
            query_embedding["lexical_weights"], doc_embed["lexical_weights"]
        )
        colbert_score = embedding_model.colbert_score(
            query_embedding["colbert_vecs"], doc_embed["colbert_vecs"]
        )
        scores["colbert"].append(colbert_score)
        scores["sparse"].append(sparse_score)
        scores["dense"].append(dense_score)

        sd_weight = weights_for_different_modes[1] + weights_for_different_modes[0]
        scores["sparse+dense"].append(
            (
                sparse_score * weights_for_different_modes[1]
                + dense_score * weights_for_different_modes[0]
            )
            / sd_weight
        )
        scores["colbert+sparse+dense"].append(
            (
                colbert_score * weights_for_different_modes[2]
                + sparse_score * weights_for_different_modes[1]
                + dense_score * weights_for_different_modes[0]
            )
            / weight_sum
        )
    return scores


def clean_path(url_path: str) -> str:
    """
    Decode and clean a URL path.
    """
    decoded = urllib.parse.unquote(url_path)
    cleaned = decoded.replace("%28", "(").replace("%29", ")").replace("%2B", "+")
    cleaned = cleaned.replace(">", " > ").replace("(", " (").replace(")", ") ")
    return " ".join(cleaned.split())


def _get_references_from_sql(
    query_embed: Dict[str, Any], current_dir: str, picklefile: str, top_k: int
) -> Tuple[List[str], List[str], List[str], List[float]]:
    """
    Retrieve top reference documents from a SQL database.
    """
    embedding_db_path = os.path.join(current_dir, "embeddings.db")
    db = sqlite3.connect(embedding_db_path)
    db.enable_load_extension(True)
    db.load_extension(EXT_VECTOR_PATH)
    db.load_extension(EXT_VSS_PATH)
    cur = db.cursor()
    query_vector = query_embed["dense_vecs"].tolist()
    query_vector_json = json.dumps(query_vector)
    cur.execute(
        """
            SELECT rowid, distance
            FROM embeddings
            WHERE vss_search(embedding, ?)
            LIMIT top_k;
        """,
        (query_vector_json,),
    )
    results = cur.fetchall()
    db.commit()
    db.close()

    table_name = picklefile.replace(".pkl", "")
    db_name = f"{table_name}.db"
    main_db_path = os.path.join(current_dir, db_name)
    db = sqlite3.connect(main_db_path)
    cur = db.cursor()
    indices = [result[0] for result in results]
    similarity_scores = [result[1] for result in results]
    placeholders = ",".join("?" for _ in indices)
    query = f"SELECT id_list, doc_list, url_list FROM {table_name} WHERE rowid IN ({placeholders})"
    cur.execute(query, indices)
    results = cur.fetchall()
    top_ids, top_docs, top_urls = [], [], []
    for id_val, doc, url in results:
        top_ids.append(id_val)
        top_docs.append(doc)
        top_urls.append(url)
    db.close()
    return top_ids, top_docs, top_urls, similarity_scores


def _get_references_from_pickle(
    query_embed: Dict[str, Any], current_dir: str, picklefile: str, top_k: int
) -> Tuple[List[str], List[str], List[str], List[float]]:
    """
    Retrieve top reference documents from a pickle file.
    """
    path_to_pickle = os.path.join(current_dir, picklefile)
    with open(path_to_pickle, "rb") as f:
        data_loaded = pickle.load(f)
    doc_list = data_loaded["doc_list"]
    file_path_list = data_loaded["file_paths_list"]
    topic_path_list = data_loaded["topic_path_list"]
    id_list = data_loaded["id_list"]
    url_list = data_loaded["url_list"]
    embedding_list = data_loaded["embedding_list"]

    combined_scores = bge_compute_score(
        query_embed, embedding_list, [1, 1, 1], None, None
    )
    score_array = np.array(combined_scores["colbert+sparse+dense"])
    indices = np.argsort(score_array)[::-1]
    similarity_scores = np.sort(score_array)[-top_k:][::-1]
    top_ids = id_list[indices][:top_k]
    top_docs = doc_list[indices][:top_k]
    top_files = file_path_list[indices][:top_k]
    top_topic_paths = topic_path_list[indices][:top_k]
    top_urls = url_list[indices][:top_k].tolist()
    return top_ids, top_docs, top_urls, similarity_scores, top_files, top_topic_paths


def _get_reference_documents(
    query_embed: Dict[str, Any], current_dir: str, picklefile: str, top_k: int
) -> Tuple[List[str], List[str], List[str], List[float]]:
    """
    Retrieve top reference documents based on the query embedding.
    """
    if SQLDB:
        return _get_references_from_sql(
            query_embed, current_dir, picklefile, top_k=top_k
        )
    else:
        return _get_references_from_pickle(
            query_embed, current_dir, picklefile, top_k=top_k
        )


def _get_pickle_and_class(course: str) -> Tuple[str, str]:
    """
    Return the pickle filename and course class name based on the course.
    """
    if course == "EE106B":
        return "eecs106b.pkl", "Robotic Manipulation and Interaction"
    elif course == "CS61A":
        return "cs61a.pkl", "Structure and Interpretation of Computer Programs"
    elif course == "CS294-137":
        return "cs294.pkl", "Immersive Computing and Virtual Reality"
    elif course == "Econ140":
        return "econ140.pkl", "Econometrics"
    elif course == "INTD315":
        return "language.pkl", "Multilingual Engagement"
    elif course == "ROAR Academy":
        return "roar_academy.pkl", "learning python and autonomous driving"
    else:
        return "Berkeley.pkl", "Berkeley"


def top_k_selector(
    message: str,
    stream: bool = True,
    rag: bool = True,
    course: Optional[str] = None,
    k: int = 3,
) -> Dict[str, Any]:
    """
    Return the top-k documents relevant to the given message.
    """
    top_docs: List[str] = []
    if rag:
        picklefile, _ = _get_pickle_and_class(course if course else "")
        current_dir = "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/"
        query_embed = embedding_model.encode(
            message, return_dense=True, return_sparse=True, return_colbert_vecs=True
        )
        if SQLDB:
            embedding_db_path = os.path.join(current_dir, "embeddings.db")
            db = sqlite3.connect(embedding_db_path)
            db.enable_load_extension(True)
            db.load_extension(EXT_VECTOR_PATH)
            db.load_extension(EXT_VSS_PATH)
            cur = db.cursor()
            query_vector = query_embed["dense_vecs"].tolist()
            query_vector_json = json.dumps(query_vector)
            cur.execute(
                """
                SELECT rowid, distance
                FROM embeddings
                WHERE vss_search(embedding, ?)
                LIMIT ?;
            """,
                (query_vector_json, k),
            )
            results = cur.fetchall()
            db.commit()
            db.close()
            table_name = picklefile.replace(".pkl", "")
            db_name = f"{table_name}.db"
            main_db_path = os.path.join(current_dir, db_name)
            db = sqlite3.connect(main_db_path)
            cur = db.cursor()
            indices = [result[0] for result in results]
            placeholders = ",".join("?" for _ in indices)
            query = f"SELECT doc_list FROM {table_name} WHERE rowid IN ({placeholders})"
            cur.execute(query, indices)
            results = cur.fetchall()
            top_docs = [doc for (doc,) in results]
            db.close()
        else:
            path_to_pickle = os.path.join(current_dir, picklefile)
            with open(path_to_pickle, "rb") as f:
                data_loaded = pickle.load(f)
            doc_list = data_loaded["doc_list"]
            embedding_list = data_loaded["embedding_list"]
            combined_scores = bge_compute_score(
                query_embed, embedding_list, [1, 1, 1], None, None
            )
            score_array = np.array(combined_scores["colbert+sparse+dense"])
            indices = np.argsort(score_array)[::-1]
            top_indices = indices[:k]
            top_docs = [doc_list[i] for i in top_indices]
    used_chunks: int = min(len(top_docs), k)
    return {"top_docs": top_docs, "used_chunks": used_chunks}
