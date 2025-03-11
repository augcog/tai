import json
import os
import pickle
import sqlite3
import urllib.parse
from threading import Thread, Lock
from typing import Any, Dict, Generator, List, Optional, Tuple

import numpy as np
import transformers
from FlagEmbedding import BGEM3FlagModel
from app.core.models.chat_completion import Message as ROARChatCompletionMessage
from dotenv import load_dotenv
from pydantic import BaseModel

# Global configuration for SQL database usage and extension paths.
SQLDB: bool = False
# TODO: Revise path design / configuration in the future for best practice.
EXT_VECTOR_PATH: str = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vector0"
EXT_VSS_PATH: str = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vss0"


# Define a Pydantic Message model.
class Message(BaseModel):
    role: str
    content: str


# Load environment variables.
load_dotenv()

# Load the embedding model.
embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)

# A threading lock for concurrency around local generation.
lock = Lock()


def prompt_generator(messages: List[Message], streamer_iterator: Any, pipeline: Any = None) -> None:
    """
    Generate text via a local Hugging Face pipeline in a background thread.
    Uses tokenizer-based prompt generation if available; otherwise calls the pipeline with raw text.

    This behavior is to enable running model on local / mock mode / remote mode.
    """
    with lock:
        if is_local_pipeline(pipeline):
            terminators = [
                pipeline.tokenizer.eos_token_id,
                pipeline.tokenizer.convert_tokens_to_ids("<|eot_id|>")
            ]
            prompt = pipeline.tokenizer.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True
            )
            pipeline(
                prompt,
                max_new_tokens=1000,
                eos_token_id=terminators,
                do_sample=True,
                streamer=streamer_iterator
            )
        else:
            prompt = messages[-1].content
            pipeline(prompt, max_new_tokens=1000, do_sample=True)


def bge_compute_score(
        query_embedding: Dict[str, Any],
        document_embeddings: List[Dict[str, Any]],
        weights_for_different_modes: Optional[List[float]] = None,
        secondary_query_embedding: Optional[Dict[str, Any]] = None,
        secondary_document_embeddings: Optional[List[Dict[str, Any]]] = None
) -> Dict[str, List[float]]:
    """
    Compute combined similarity scores (dense, sparse, colbert, and weighted combinations).
    Returns a dictionary of lists for each scoring mode.
    """
    scores: Dict[str, List[float]] = {
        'colbert': [],
        'sparse': [],
        'dense': [],
        'sparse+dense': [],
        'colbert+sparse+dense': [],
    }
    if weights_for_different_modes is None:
        weights_for_different_modes = [1, 1.0, 1.0]
        weight_sum = 3
        print("Default weights for dense, sparse, colbert are [1.0, 1.0, 1.0]")
    else:
        assert len(weights_for_different_modes) == 3
        weight_sum = sum(weights_for_different_modes)

    for doc_embed in document_embeddings:
        dense_score = query_embedding['dense_vecs'] @ doc_embed['dense_vecs'].T
        sparse_score = embedding_model.compute_lexical_matching_score(
            query_embedding['lexical_weights'], doc_embed['lexical_weights']
        )
        colbert_score = embedding_model.colbert_score(
            query_embedding['colbert_vecs'], doc_embed['colbert_vecs']
        )
        scores['colbert'].append(colbert_score)
        scores['sparse'].append(sparse_score)
        scores['dense'].append(dense_score)

        sd_weight = weights_for_different_modes[1] + weights_for_different_modes[0]
        scores['sparse+dense'].append(
            (sparse_score * weights_for_different_modes[1] +
             dense_score * weights_for_different_modes[0]) / sd_weight
        )
        scores['colbert+sparse+dense'].append(
            (colbert_score * weights_for_different_modes[2] +
             sparse_score * weights_for_different_modes[1] +
             dense_score * weights_for_different_modes[0]) / weight_sum
        )
    return scores


def clean_path(url_path: str) -> str:
    """
    Decode and clean a URL path.
    """
    decoded = urllib.parse.unquote(url_path)
    cleaned = decoded.replace('%28', '(').replace('%29', ')').replace('%2B', '+')
    cleaned = cleaned.replace('>', ' > ').replace('(', ' (').replace(')', ') ')
    return ' '.join(cleaned.split())


def _get_references_from_sql(query_embed: Dict[str, Any], current_dir: str, picklefile: str
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
    query_vector = query_embed['dense_vecs'].tolist()
    query_vector_json = json.dumps(query_vector)
    cur.execute("""
            SELECT rowid, distance
            FROM embeddings
            WHERE vss_search(embedding, ?)
            LIMIT 3;
        """, (query_vector_json,))
    results = cur.fetchall()
    db.commit()
    db.close()

    table_name = picklefile.replace('.pkl', '')
    db_name = f"{table_name}.db"
    main_db_path = os.path.join(current_dir, db_name)
    db = sqlite3.connect(main_db_path)
    cur = db.cursor()
    indices = [result[0] for result in results]
    similarity_scores = [result[1] for result in results]
    placeholders = ','.join('?' for _ in indices)
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


def _get_references_from_pickle(query_embed: Dict[str, Any], current_dir: str, picklefile: str
                                ) -> Tuple[List[str], List[str], List[str], List[float]]:
    """
    Retrieve top reference documents from a pickle file.
    """
    path_to_pickle = os.path.join(current_dir, picklefile)
    with open(path_to_pickle, 'rb') as f:
        data_loaded = pickle.load(f)
    doc_list = data_loaded['doc_list']
    id_list = data_loaded['id_list']
    url_list = data_loaded['url_list']
    embedding_list = data_loaded['embedding_list']

    combined_scores = bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)
    score_array = np.array(combined_scores['colbert+sparse+dense'])
    indices = np.argsort(score_array)[::-1]
    similarity_scores = np.sort(score_array)[-3:][::-1]
    top_ids = id_list[indices][:3]
    top_docs = doc_list[indices][:3]
    top_urls = url_list[indices][:3].tolist()
    return top_ids, top_docs, top_urls, similarity_scores


def _get_reference_documents(query_embed: Dict[str, Any], current_dir: str, picklefile: str
                             ) -> Tuple[List[str], List[str], List[str], List[float]]:
    """
    Retrieve top reference documents based on the query embedding.
    """
    if SQLDB:
        return _get_references_from_sql(query_embed, current_dir, picklefile)
    else:
        return _get_references_from_pickle(query_embed, current_dir, picklefile)


def _get_pickle_and_class(course: str) -> Tuple[str, str]:
    """
    Return the pickle filename and course class name based on the course.
    """
    if course == "EE 106B":
        return "eecs106b.pkl", "Robotic Manipulation and Interaction"
    elif course == "CS 61A":
        return "cs61a.pkl", "Structure and Interpretation of Computer Programs"
    elif course == "CS 294-137":
        return "cs294.pkl", "Immersive Computing and Virtual Reality"
    elif course == "Econ 140":
        return "Econ140.pkl", "Econometrics"
    else:
        return "Berkeley.pkl", "Berkeley"


def build_augmented_message(user_message: str, course: str, embedding_dir: str, threshold: float, rag: bool
                            ) -> Tuple[str, List[str], str]:
    """
    Build an augmented prompt by retrieving reference documents.
    Returns:
      - modified_message: the augmented instruction prompt.
      - reference_list: list of reference URLs for JSON output.
      - reference_string: formatted string for plain text references.
    TODO: reference_string can be removed in the future once the legacy code migration is completed.
    """
    if not rag:
        return user_message, [], ""

    picklefile, class_name = _get_pickle_and_class(course)
    current_dir = embedding_dir
    query_embed = embedding_model.encode(
        user_message, return_dense=True, return_sparse=True, return_colbert_vecs=True
    )
    top_ids, top_docs, top_urls, similarity_scores = _get_reference_documents(query_embed, current_dir, picklefile)

    insert_document = ""
    reference_list: List[str] = []
    reference_string = ""
    n = 0

    for i in range(len(top_docs)):
        reference_list.append(top_urls[i] if top_urls[i] else "")
        if similarity_scores[i] > threshold:
            n += 1
            cleaned = clean_path(top_ids[i])
            if top_urls[i]:
                insert_document += (
                    f"\"\"\"Reference Number: {n}\n"
                    f"Reference Info Path: {top_ids[i]}\n"
                    f"Reference_Url: {top_urls[i]}\n"
                    f"Document: {top_docs[i]}\"\"\"\n\n"
                )
                reference_string += (
                    f"Reference {n}: <|begin_of_reference_name|>{cleaned}"
                    f"<|end_of_reference_name|><|begin_of_reference_link|>{top_urls[i]}"
                    f"<|end_of_reference_link|>\n\n"
                )
            else:
                insert_document += (
                    f"\"\"\"Reference Number: {n}\n"
                    f"Reference Info Path: {cleaned}\n"
                    f"Reference_Url: NONE\n"
                    f"Document: {top_docs[i]}\"\"\"\n\n"
                )
                reference_string += (
                    f"Reference {n}: <|begin_of_reference_name|>{cleaned}"
                    f"<|end_of_reference_name|><|begin_of_reference_link|>"
                    f"<|end_of_reference_link|>\n\n"
                )

    if not insert_document or n == 0:
        modified_message = (
            f"Answer the instruction. If unsure of the answer, explain that there is no data in the knowledge base "
            f"for the response and refuse to answer. If the instruction is not related to class topic {class_name}, "
            f"explain and refuse to answer.\n---\n"
            f"Instruction: {user_message}"
        )
    else:
        insert_document += f"Instruction: {user_message}"
        modified_message = (
            f"Understand the reference documents and use related ones to answer the instruction thoroughly. "
            f"Keep your answer grounded in the facts of the references that are relevant and refer to specific "
            f"reference number inline. Do not provide any reference at the end. "
            f"If the instruction is not related to class topic {class_name}, explain and refuse to answer.\n"
            f"---\n{insert_document}"
        )
    return modified_message, reference_list, reference_string


def local_parser(stream: Any, reference_string: str) -> Generator[str, None, None]:
    """
    Yield tokens from a text stream and append the reference block at the end.
    TODO: This function can be removed in the future once the legacy code migration is completed.
    """
    for chunk in stream:
        result = chunk.replace("<|eot_id|>", "")
        yield result if result is not None else ""
        print(result, end="")
    ref_block = f'\n\n<|begin_of_reference|>\n\n{reference_string}<|end_of_reference|>'
    yield ref_block
    print(ref_block)


def local_parser_2(stream: Any) -> Generator[str, None, None]:
    """
    Yield tokens from a text stream (simplified version for JSON output).
    """
    for chunk in stream:
        result = chunk.replace("<|eot_id|>", "")
        yield result if result is not None else ""


def local_formatter(messages: List[ROARChatCompletionMessage]) -> List[Message]:
    """
    Format a conversation by prepending an initial system message.
    """
    response: List[Message] = []
    system_message = (
        "You are a Teaching Assistant. You are responsible for answering questions and providing guidance to students. "
        "Do not provide direct answers to homework questions. If the question is related to a class topic, "
        "provide guidance and resources to help the student answer the question. "
        "If the question is not related to a class topic, explain that you cannot provide an answer."
    )
    response.append(Message(role="system", content=system_message))
    for message in messages:
        response.append(Message(role=message.role, content=message.content))
    return response


def top_k_selector(message: str, stream: bool = True, rag: bool = True, course: Optional[str] = None, k: int = 3
                   ) -> Dict[str, Any]:
    """
    Return the top-k documents relevant to the given message.
    """
    top_docs: List[str] = []
    if rag:
        picklefile, _ = _get_pickle_and_class(course if course else "")
        current_dir = "/home/bot/localgpt/tai_evaluation/tai/rag/file_conversion_router/embedding"
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
            query_vector = query_embed['dense_vecs'].tolist()
            query_vector_json = json.dumps(query_vector)
            cur.execute("""
                SELECT rowid, distance
                FROM embeddings
                WHERE vss_search(embedding, ?)
                LIMIT ?;
            """, (query_vector_json, k))
            results = cur.fetchall()
            db.commit()
            db.close()
            table_name = picklefile.replace('.pkl', '')
            db_name = f"{table_name}.db"
            main_db_path = os.path.join(current_dir, db_name)
            db = sqlite3.connect(main_db_path)
            cur = db.cursor()
            indices = [result[0] for result in results]
            placeholders = ','.join('?' for _ in indices)
            query = f"SELECT doc_list FROM {table_name} WHERE rowid IN ({placeholders})"
            cur.execute(query, indices)
            results = cur.fetchall()
            top_docs = [doc for (doc,) in results]
            db.close()
        else:
            path_to_pickle = os.path.join(current_dir, picklefile)
            with open(path_to_pickle, 'rb') as f:
                data_loaded = pickle.load(f)
            doc_list = data_loaded['doc_list']
            embedding_list = data_loaded['embedding_list']
            combined_scores = bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)
            score_array = np.array(combined_scores['colbert+sparse+dense'])
            indices = np.argsort(score_array)[::-1]
            top_indices = indices[:k]
            top_docs = [doc_list[i] for i in top_indices]
    used_chunks: int = min(len(top_docs), k)
    return {"top_docs": top_docs, "used_chunks": used_chunks}


def local_selector(
        messages: List[Message],
        stream: bool = True,
        rag: bool = True,
        course: Optional[str] = None,
        embedding_dir: str = "/home/bot/localgpt/tai/ai_course_bot/ai-chatbot-backend/app/embedding/",
        threshold: float = 0.45,
        pipeline: Any = None
) -> Tuple[Any, str]:
    """
    Build an augmented message with references and run LLM inference.
    Returns a tuple: (stream, reference_string)
    """
    user_message = messages[-1].content
    modified_message, _, reference_string = build_augmented_message(
        user_message, course if course else "", embedding_dir, threshold, rag
    )
    messages[-1].content = modified_message
    if is_local_pipeline(pipeline):
        streamer_iterator = transformers.TextIteratorStreamer(pipeline.tokenizer, skip_prompt=True)
        t = Thread(target=prompt_generator, args=(messages, streamer_iterator, pipeline))
        t.start()
        return streamer_iterator, reference_string
    else:
        response = pipeline(messages[-1].content, stream=stream, course=course)
        return response, reference_string


def local_selector_with_json_generator(
        messages: List[Message],
        stream: bool = True,
        rag: bool = True,
        course: Optional[str] = None,
        # TODO: Revise the default embedding_dir path. And put it into the environment variable for best practice.
        embedding_dir: str = "/home/bot/localgpt/tai/ai_course_bot/ai-chatbot-backend/app/embedding/",
        threshold: float = 0.45,
        pipeline: Any = None
) -> Generator[str, None, None]:
    """
    Build an augmented message with references and produce a JSON streaming response.
    """
    user_message = messages[-1].content
    modified_message, reference_list, reference_string = build_augmented_message(
        user_message, course if course else "", embedding_dir, threshold, rag
    )
    messages[-1].content = modified_message
    if is_local_pipeline(pipeline):
        streamer_iterator = transformers.TextIteratorStreamer(pipeline.tokenizer, skip_prompt=True)
        t = Thread(target=prompt_generator, args=(messages, streamer_iterator, pipeline))
        t.start()

        def stream_json_response() -> Generator[str, None, None]:
            for chunk in local_parser_2(streamer_iterator):
                yield json.dumps({"type": "token", "data": chunk}) + "\n"
            yield json.dumps({"type": "final", "references": reference_list}) + "\n"

        return stream_json_response()
    else:
        remote_stream = pipeline(messages[-1].content, stream=stream, course=course)

        def stream_json_response() -> Generator[str, None, None]:
            for item in remote_stream:
                yield item
            yield json.dumps({"type": "final", "references": reference_list}) + "\n"

        return stream_json_response()


def is_local_pipeline(pipeline: Any) -> bool:
    """Return True if the pipeline is local (has a tokenizer), else False."""
    return hasattr(pipeline, "tokenizer") and pipeline.tokenizer is not None
