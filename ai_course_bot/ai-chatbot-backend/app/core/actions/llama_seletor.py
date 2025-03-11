import json
import os
import pickle
import sqlite3
import urllib.parse
from threading import Thread, Lock
from typing import List

import numpy as np
import transformers
from FlagEmbedding import BGEM3FlagModel
from app.core.models.chat_completion import Message as ROARChatCompletionMessage
from dotenv import load_dotenv
from pydantic import BaseModel

# Global configuration for SQL database usage and extension paths.
SQLDB = False
EXT_VECTOR_PATH = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vector0"
EXT_VSS_PATH = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vss0"


# Define a Pydantic Message model.
class Message(BaseModel):
    role: str
    content: str


# Load the embedding model.
embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)

# Load environment variables.
load_dotenv()

# A threading lock for concurrency around local generation.
lock = Lock()


def prompt_generator(messages: List[Message], streamer_iterator, pipeline=None):
    """
    Generate text via a local HF pipeline in a separate thread.
    If the pipeline has a tokenizer, we run local generation; otherwise,
    we simply call pipeline(...) as is (intended for remote or mock).
    """
    with lock:
        if hasattr(pipeline, "tokenizer") and pipeline.tokenizer is not None:
            # Local pipeline logic
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
            # Remote/mock pipeline: pass raw last user content
            prompt = messages[-1].content
            # For remote/mock, assume pipeline returns a generator or final response
            pipeline(prompt, max_new_tokens=1000, do_sample=True)


def bge_compute_score(
        query_embedding,
        document_embeddings,
        weights_for_different_modes,
        secondary_query_embedding,
        secondary_document_embeddings,
):
    """
    Compute scores across different modes (dense, sparse, colbert, and their combinations).
    """
    all_scores = {
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

    for i in range(len(document_embeddings)):
        dense_score = query_embedding['dense_vecs'] @ document_embeddings[i]['dense_vecs'].T
        sparse_score = embedding_model.compute_lexical_matching_score(
            query_embedding['lexical_weights'],
            document_embeddings[i]['lexical_weights']
        )
        colbert_score = embedding_model.colbert_score(
            query_embedding['colbert_vecs'],
            document_embeddings[i]['colbert_vecs']
        )
        all_scores['colbert'].append(colbert_score)
        all_scores['sparse'].append(sparse_score)
        all_scores['dense'].append(dense_score)

        # Weighted combos
        sd_weight = weights_for_different_modes[1] + weights_for_different_modes[0]
        all_scores['sparse+dense'].append(
            (sparse_score * weights_for_different_modes[1] + dense_score * weights_for_different_modes[0]) / sd_weight
        )
        all_scores['colbert+sparse+dense'].append(
            (colbert_score * weights_for_different_modes[2] +
             sparse_score * weights_for_different_modes[1] +
             dense_score * weights_for_different_modes[0]) / weight_sum
        )

    return all_scores


def clean_path(url_path):
    """
    Clean and decode a URL path.
    """
    decoded_path = urllib.parse.unquote(url_path)
    cleaned_path = decoded_path.replace('%28', '(').replace('%29', ')').replace('%2B', '+')
    cleaned_path = cleaned_path.replace('>', ' > ')
    cleaned_path = cleaned_path.replace('(', ' (').replace(')', ') ')
    cleaned_path = ' '.join(cleaned_path.split())
    return cleaned_path


def _get_reference_documents(query_embed, current_dir, picklefile):
    """
    Retrieve top reference documents based on the query embedding.
    Returns top_ids, top_docs, top_urls, and distances.
    """
    if SQLDB:
        # SQL branch: Connect to the embeddings DB, query top results,
        # then connect to the main DB to extract details.
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
            WHERE vss_search(
                embedding,
                ?
            )
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
        distances = [result[1] for result in results]
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
    else:
        # Pickle branch: Load the precomputed data.
        path_to_pickle = os.path.join(current_dir, picklefile)
        with open(path_to_pickle, 'rb') as f:
            data_loaded = pickle.load(f)
        doc_list = data_loaded['doc_list']
        id_list = data_loaded['id_list']
        url_list = data_loaded['url_list']
        embedding_list = data_loaded['embedding_list']

        # Weighted scoring
        combined_scores = bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)
        score_array = np.array(combined_scores['colbert+sparse+dense'])

        indices = np.argsort(score_array)[::-1]
        distances = np.sort(score_array)[-3:][::-1]

        top_ids = id_list[indices][:3]
        top_docs = doc_list[indices][:3]
        top_urls = url_list[indices][:3].tolist()

    return top_ids, top_docs, top_urls, distances


def build_augmented_message(user_message: str, course: str, embedding_dir: str, threshold: float, rag: bool):
    """
    Build the augmented user message by retrieving reference documents and appending reference details.
    Returns a tuple: (modified_message, reference_list, reference_string)
      - modified_message: the augmented instruction prompt.
      - reference_list: list of reference URLs (or empty strings) for JSON output.
      - reference_string: formatted reference string (for plain text output).
    """
    if not rag:
        return user_message, [], ""

    # Determine the pickle file and class name
    if course == "EE 106B":
        picklefile = "eecs106b.pkl"
        class_name = 'Robotic Manipulation and Interaction'
    elif course == "CS 61A":
        picklefile = "cs61a.pkl"
        class_name = 'Structure and Interpretation of Computer Programs'
    elif course == "CS 294-137":
        picklefile = "cs294.pkl"
        class_name = 'Immersive Computing and Virtual Reality'
    elif course == "Econ 140":
        picklefile = "Econ140.pkl"
        class_name = 'Econometrics'
    else:
        picklefile = "Berkeley.pkl"
        class_name = 'Berkeley'

    current_dir = embedding_dir

    # Encode the query for similarity
    query_embed = embedding_model.encode(
        user_message,
        return_dense=True,
        return_sparse=True,
        return_colbert_vecs=True
    )

    top_ids, top_docs, top_urls, distances = _get_reference_documents(query_embed, current_dir, picklefile)

    insert_document = ""
    reference_list = []
    reference_string = ""
    n = 0

    for i in range(len(top_docs)):
        # Always append some reference (URL or empty) for JSON output
        reference_list.append(top_urls[i] if top_urls[i] else "")

        # Only append relevant docs if distance above threshold
        if distances[i] > threshold:
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
                    f"Reference {n}: "
                    f"<|begin_of_reference_name|>{cleaned}"
                    f"<|end_of_reference_name|>"
                    f"<|begin_of_reference_link|>{top_urls[i]}"
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

    # If no relevant references pass threshold, just mention that there's no data.
    if (not insert_document) or n == 0:
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


def local_parser(stream, reference_string: str):
    """
    Yields tokens from a text stream (from the local pipeline) and finally yields
    the reference string at the end.
    """
    for chunk in stream:
        result = chunk.replace("<|eot_id|>", "")
        if result is None:
            yield ""
        else:
            yield result
            print(result, end="")

    # Once the stream completes, output references
    ref_block = f'\n\n<|begin_of_reference|>\n\n{reference_string}<|end_of_reference|>'
    yield ref_block
    print(ref_block)


def local_parser_2(stream):
    """
    Yields tokens from a text stream (from the local pipeline) in a simpler form,
    suitable for JSON streaming usage.
    """
    for chunk in stream:
        result = chunk.replace("<|eot_id|>", "")
        if result is None:
            yield ""
        else:
            yield result


def local_formatter(messages: List[ROARChatCompletionMessage]) -> List[Message]:
    """
    Format the conversation with an initial system message for guidance.
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


def top_k_selector(message: str, stream=True, rag=True, course=None, k=3):
    """
    A helper function to return the top-k documents relevant to the message.
    """
    top_docs = []
    if rag:
        if course == "EE 106B":
            picklefile = "eecs106b.pkl"
        elif course == "CS 61A":
            picklefile = "cs61a.pkl"
        else:
            picklefile = "Berkeley.pkl"

        current_dir = "/home/bot/localgpt/tai_evaluation/tai/rag/file_conversion_router/embedding"
        query_embed = embedding_model.encode(
            message,
            return_dense=True,
            return_sparse=True,
            return_colbert_vecs=True
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
                SELECT 
                    rowid, 
                    distance
                FROM embeddings
                WHERE vss_search(
                    embedding,
                    ?
                )
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

    used_chunks = min(len(top_docs), k)
    return {
        "top_docs": top_docs,
        "used_chunks": used_chunks
    }


def local_selector(
        messages: List[Message],
        stream=True,
        rag=True,
        course=None,
        embedding_dir: str = "/home/bot/localgpt/tai/ai_course_bot/ai-chatbot-backend/app/embedding/",
        threshold: float = 0.45,
        pipeline=None
):
    """
    Build an augmented message with references, then run LLM inference.
    If pipeline has a tokenizer, we spawn a local streaming generator.
    Otherwise (mock/remote pipeline), we return whatever the pipeline gives.

    Returns:
        - A "stream" object (either a TextIteratorStreamer or the pipeline's response).
        - The reference string for appending at the end (if needed).
    """
    # Always build references first
    user_message = messages[-1].content
    modified_message, _, reference_string = build_augmented_message(
        user_message, course, embedding_dir, threshold, rag
    )
    messages[-1].content = modified_message

    # Now handle local vs mock/remote
    if hasattr(pipeline, "tokenizer") and pipeline.tokenizer is not None:
        # Local pipeline path
        streamer_iterator = transformers.TextIteratorStreamer(pipeline.tokenizer, skip_prompt=True)
        t = Thread(target=prompt_generator, args=(messages, streamer_iterator, pipeline))
        t.start()
        return streamer_iterator, reference_string
    else:
        # Mock/remote pipeline path (no tokenizer)
        # Typically returns a generator or final response
        response = pipeline(messages[-1].content, stream=stream, course=course)
        return response, reference_string


def local_selector_with_json_generator(
        messages: List[Message],
        stream=True,
        rag=True,
        course=None,
        embedding_dir: str = "/home/bot/localgpt/tai/ai_course_bot/ai-chatbot-backend/app/embedding/",
        threshold: float = 0.45,
        pipeline=None
):
    """
    Builds an augmented message with references. If the pipeline has a tokenizer (local),
    we do local streaming of tokens, then yield references as final JSON.
    If the pipeline has no tokenizer (mock/remote), we call pipeline(...) directly,
    but still do the reference-chunk logic first.
    We then produce a JSON streaming response with tokens followed by references.
    """
    # Always do chunk/references logic
    user_message = messages[-1].content
    modified_message, reference_list, reference_string = build_augmented_message(
        user_message, course, embedding_dir, threshold, rag
    )
    messages[-1].content = modified_message

    if hasattr(pipeline, "tokenizer") and pipeline.tokenizer is not None:
        # Local pipeline streaming
        streamer_iterator = transformers.TextIteratorStreamer(pipeline.tokenizer, skip_prompt=True)
        t = Thread(target=prompt_generator, args=(messages, streamer_iterator, pipeline))
        t.start()

        def stream_json_response():
            # Stream out tokens in JSON
            for chunk in local_parser_2(streamer_iterator):
                yield json.dumps({"type": "token", "data": chunk}) + "\n"
            # Then yield final references
            yield json.dumps({"type": "final", "references": reference_list}) + "\n"

        return stream_json_response()
    else:
        # Mock/remote pipeline streaming
        # The pipeline call typically yields JSON lines that we want to pass along.
        # We still want to append references at the end in our own format.
        remote_stream = pipeline(messages[-1].content, stream=stream, course=course)

        def stream_json_response():
            # Pass through the remote pipeline tokens
            for item in remote_stream:
                yield item
            # Then yield final references
            yield json.dumps({"type": "final", "references": reference_list}) + "\n"

        return stream_json_response()
