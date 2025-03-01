import os
import pickle
import numpy as np
from dotenv import load_dotenv
from FlagEmbedding import BGEM3FlagModel
import transformers
import torch
from threading import Thread
from typing import List
from app.core.models.chat_completion import Message as ROARChatCompletionMessage
from pydantic import BaseModel
import threading
import urllib.parse
import json
import sys
import sqlite3

# Set the environment variable to use the SQL database
SQLDB = False
EXT_VECTOR_PATH = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vector0"
EXT_VSS_PATH = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vss0"


class Message(BaseModel):
    role: str
    content: str


# lode embedding model
embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)

load_dotenv()

model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
auto_tokenizer = transformers.AutoTokenizer.from_pretrained(model_id)

print("Loading model...")
pipeline = transformers.pipeline(
    "text-generation",
    model=model_id,
    model_kwargs={"torch_dtype": torch.bfloat16},
    device="cuda",
)

lock = threading.Lock()


def prompt_generator(messages, streamer_iterator):
    with lock:
        terminators = [
            pipeline.tokenizer.eos_token_id,
            pipeline.tokenizer.convert_tokens_to_ids("<|eot_id|>")
        ]
        prompt = pipeline.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )

        outputs = pipeline(
            prompt,
            max_new_tokens=1000,
            eos_token_id=terminators,
            do_sample=True,
            streamer=streamer_iterator
        )


# Write scores for
def bge_compute_score(
        query_embedding,
        document_embeddings,
        weights_for_different_modes,
        secondary_query_embedding,
        secondary_document_embeddings,
):
    all_scores = {
        'colbert': [],
        'sparse': [],
        'dense': [],
        'sparse+dense': [],
        'colbert+sparse+dense': [],
    }

    if weights_for_different_modes is None:
        weights_for_different_modes = [1, 1., 1.]
        weight_sum = 3
        print("default weights for dense, sparse, colbert are [1.0, 1.0, 1.0]")
    else:
        assert len(weights_for_different_modes) == 3
        weight_sum = sum(weights_for_different_modes)

    # Loop through each document embedding
    for i in range(len(document_embeddings)):
        dense_score = query_embedding['dense_vecs'] @ document_embeddings[i]['dense_vecs'].T
        sparse_score = embedding_model.compute_lexical_matching_score(query_embedding['lexical_weights'],
                                                                      document_embeddings[i]['lexical_weights'])
        colbert_score = embedding_model.colbert_score(query_embedding['colbert_vecs'],
                                                      document_embeddings[i]['colbert_vecs'])
        # Store the scores
        all_scores['colbert'].append(colbert_score)
        all_scores['sparse'].append(sparse_score)
        all_scores['dense'].append(dense_score)
        all_scores['sparse+dense'].append(
            (sparse_score * weights_for_different_modes[1] + dense_score * weights_for_different_modes[0]) /
            (weights_for_different_modes[1] + weights_for_different_modes[0])
        )
        all_scores['colbert+sparse+dense'].append(
            (colbert_score * weights_for_different_modes[2] + sparse_score * weights_for_different_modes[1] +
             dense_score * weights_for_different_modes[0]) / weight_sum
        )

    return all_scores


def clean_path(url_path):
    decoded_path = urllib.parse.unquote(url_path)
    cleaned_path = decoded_path.replace('%28', '(').replace('%29', ')').replace('%2B', '+')
    cleaned_path = cleaned_path.replace('>', ' > ')
    cleaned_path = cleaned_path.replace('(', ' (').replace(')', ') ')
    cleaned_path = ' '.join(cleaned_path.split())
    return cleaned_path


def local_selector(messages: List[Message], stream=True, rag=True, course=None):
    insert_document = ""
    user_message = messages[-1].content
    if rag:
        if course == "EE 106B":
            picklefile = "eecs106b.pkl"
            class_name = 'Robotic Manipulation and Interaction'
        elif course == "CS 61A":
            picklefile = "cs61a.pkl"
            class_name = 'Structure and Interpretation of Computer Programs'
        elif course == "CS 294-137":
            picklefile = "cs294_old.pkl"
            class_name = 'Immersive Computing and Virtual Reality'
        elif course == "Econ 140":
            picklefile = "econ140.pkl"
            class_name = 'Econometrics'
        else:
            picklefile = "Berkeley.pkl"
            class_name = 'Berkeley'
        current_dir = "/home/bot/localgpt/tai/ai_course_bot/ai-chatbot-backend/app/embedding/"  # Modify this path to the directory containing the embedding pickle files
        query_embed = embedding_model.encode(user_message, return_dense=True, return_sparse=True,
                                             return_colbert_vecs=True)
        if SQLDB:
            embedding_db_path = os.path.join(current_dir, "embeddings.db")

            # Connect to the embeddings database using vss and vector extensions
            db = sqlite3.connect(embedding_db_path)
            db.enable_load_extension(True)
            db.load_extension(EXT_VECTOR_PATH)
            db.load_extension(EXT_VSS_PATH)
            cur = db.cursor()

            # Query the embeddings database using vss_search
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
                LIMIT 3;
            """, (query_vector_json,))

            results = cur.fetchall()

            # Close the connection
            db.commit()
            db.close()

            # Connect to the main database to extract the top docs and urls
            table_name = picklefile.replace('.pkl', '')
            db_name = f"{table_name}.db"
            main_db_path = os.path.join(current_dir, db_name)
            db = sqlite3.connect(main_db_path)
            cur = db.cursor()

            # Extract the top 3 docs and urls
            indices = [result[0] for result in results]
            distances = [result[1] for result in results]

            placeholders = ','.join('?' for _ in indices)
            query = f"SELECT id_list, doc_list, url_list FROM {table_name} WHERE rowid IN ({placeholders})"
            cur.execute(query, indices)
            results = cur.fetchall()

            top_ids, top_docs, top_urls = [], [], []
            for id, doc, url in results:
                top_ids.append(id)
                top_docs.append(doc)
                top_urls.append(url)

            # Close the connection
            db.close()

        else:
            # Picklefile implementation
            path_to_pickle = os.path.join(current_dir, picklefile)
            with open(path_to_pickle, 'rb') as f:
                data_loaded = pickle.load(f)

            doc_list = data_loaded['doc_list']
            id_list = data_loaded['id_list']
            url_list = data_loaded['url_list']
            embedding_list = data_loaded['embedding_list']

            cosine_similarities = np.array(
                bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)['colbert+sparse+dense'])
            indices = np.argsort(cosine_similarities)[::-1]
            distances = np.sort(cosine_similarities)[-3:][::-1]
            print("indices:", indices, "distances:", distances)
            top_ids = id_list[indices][:3]
            top_docs = doc_list[indices][:3]
            top_urls = url_list[indices][:3]

        print("top_ids:", top_ids, "top_docs:", top_docs, "top_urls:", top_urls)

        insert_document = ""
        reference = []
        n = 0
        reference_string = ""
        for i in range(len(top_docs)):
            if top_urls[i]:
                reference.append(f"{top_urls[i]}")
            else:
                reference.append("")
            if distances[i] > 0.38:
                n += 1
                if top_urls[i]:
                    insert_document += f"\"\"\"Reference Number: {n}\nReference Info Path(not URL): {top_ids[i]}\nDocument: {top_docs[i]}\"\"\"\n\n"
                    cleaned_path = clean_path(top_ids[i])
                    reference_string += f"Reference {n}: <|begin_of_reference_name|>{cleaned_path}<|end_of_reference_name|><|begin_of_reference_link|>{top_urls[i]}<|end_of_reference_link|>\n\n"
                else:
                    cleaned_path = clean_path(top_ids[i])
                    insert_document += f"\"\"\"Reference Number: {n}\nReference Info Path(not URL): {cleaned_path}\nDocument: {top_docs[i]}\"\"\"\n\n"
                    reference_string += f"Reference {n}: <|begin_of_reference_name|>{cleaned_path}<|end_of_reference_name|><|begin_of_reference_link|><|end_of_reference_link|>\n\n"
            else:
                reference.append("")

        print(reference)

    if (not insert_document) or n == 0:
        print("NO REFERENCES")
        user_message = f'Answer the instruction. If unsure of the answer, explain that there is no data in the knowledge base for the response and refuse to answer. If the instruction is not related to class topic {class_name}, explain and refuse to answer.\n---\nInstruction: {user_message}'
    else:
        print("INSERT DOCUMENT", insert_document)
        insert_document += f'Instruction: {user_message}'
        # user_message = f"Understand the reference documents and use them to answer the instruction thoroughly. List the references used to answer the question numbered. Ex: [reference Name](URL). Keep your answer ground in the facts of the references.  \n---\n{insert_document}"
        user_message = f"Understand the reference documents and use related ones to answer the instruction thoroughly. Keep your answer ground in the facts of the references that are relevant and refer to spacific reference number inline. Do not provide any reference at the end. If the instruction is not related to class topic {class_name}, explain and refuse to answer.\n---\n{insert_document}"

    print("USER MESSAGE", user_message)
    messages[-1].content = user_message

    # print(messages)
    streamer_iterator = transformers.TextIteratorStreamer(auto_tokenizer, skip_prompt=True)
    t = Thread(target=prompt_generator, args=(messages, streamer_iterator,))
    t.start()
    response = streamer_iterator
    return response, reference_string


def local_parser(stream, reference_string):
    for chunk in stream:
        result = chunk.replace("<|eot_id|>", "")
        if result == None:
            yield ""
        else:
            yield chunk.replace("<|eot_id|>", "")
            print(chunk.replace("<|eot_id|>", ""), end="")
    yield '\n\n<|begin_of_reference|>\n\n' + reference_string + '<|end_of_reference|>'
    print('\n\n<|begin_of_reference|>\n\n' + reference_string + '<|end_of_reference|>')


def local_formatter(messages: List[ROARChatCompletionMessage]) -> List[Message]:
    response: List[Message] = []  # type: ignore
    system_message = "You are a Teaching Assistant. You are responsible for answering questions and providing guidance to students. Do not provide direct answers to homework questions. If the question is related to a class topic, provide guidance and resources to help the student answer the question. If the question is not related to a class topic, explain that you cannot provide an answer."
    response.append(Message(role="system", content=system_message))
    for message in messages:
        response.append(Message(role=message.role, content=message.content))
    return response


def top_k_selector(message: str, stream=True, rag=True, course=None, k=3):
    user_message = message
    top_docs = []

    if rag:
        if course == "EE 106B":
            picklefile = "eecs106b.pkl"

        elif course == "CS 61A":
            picklefile = "cs61a.pkl"
        else:
            picklefile = "Berkeley.pkl"
        current_dir = "/home/bot/localgpt/tai_evaluation/tai/rag/file_conversion_router/embedding"  # Modify this path to the directory containing the embedding pickle files
        query_embed = embedding_model.encode(user_message, return_dense=True, return_sparse=True,
                                             return_colbert_vecs=True)
        if SQLDB:
            embedding_db_path = os.path.join(current_dir, "embeddings.db")

            # Connect to the embeddings database using vss and vector extensions
            db = sqlite3.connect(embedding_db_path)
            db.enable_load_extension(True)
            db.load_extension(EXT_VECTOR_PATH)
            db.load_extension(EXT_VSS_PATH)
            cur = db.cursor()

            # Query the embeddings database using vss_search
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

            # Close the connection
            db.commit()
            db.close()

            # Connect to the main database to extract the top docs
            table_name = picklefile.replace('.pkl', '')
            db_name = f"{table_name}.db"
            main_db_path = os.path.join(current_dir, db_name)
            db = sqlite3.connect(main_db_path)
            cur = db.cursor()

            # Extract the top k docs
            indices = [result[0] for result in results]
            placeholders = ','.join('?' for _ in indices)
            query = f"SELECT doc_list FROM {table_name} WHERE rowid IN ({placeholders})"
            cur.execute(query, indices)
            results = cur.fetchall()

            top_docs = [doc for (doc,) in results]  # Extract docs from results

            # Close the connection
            db.close()

        else:
            # Picklefile implementation
            path_to_pickle = os.path.join(current_dir, picklefile)
            with open(path_to_pickle, 'rb') as f:
                data_loaded = pickle.load(f)

            doc_list = data_loaded['doc_list']
            embedding_list = data_loaded['embedding_list']

            cosine_similarities = np.array(
                bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)['colbert+sparse+dense'])
            indices = np.argsort(cosine_similarities)[::-1]
            top_indices = indices[:k]  # Get top k
            top_docs = [doc_list[i] for i in top_indices]  # Extract top k docs

    used_chunks = min(len(top_docs), k)

    # Return the top k documents as a list of strings
    return {
        "top_docs": top_docs,
        "used_chunks": used_chunks
    }