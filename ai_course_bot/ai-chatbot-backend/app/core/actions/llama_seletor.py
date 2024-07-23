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
import sqlite3
import json
from app.embedding.table_create import execute_all, connect, insert

# Set the environment variable to use the SQL database
SQLDB = False

class Message(BaseModel):
    role: str
    content: str

# lode embedding model
embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)

load_dotenv()

model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
auto_tokenizer = transformers.AutoTokenizer.from_pretrained(model_id)
# streamer_iterator = transformers.TextIteratorStreamer(auto_tokenizer, skip_prompt=True)
print("Loading model...")
pipeline = transformers.pipeline(
    "text-generation",
    model=model_id,
    model_kwargs={"torch_dtype": torch.bfloat16},
    device="cuda",
)

lock = threading.Lock()
def prompt_generator(messages,streamer_iterator):
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
        sparse_score = embedding_model.compute_lexical_matching_score(query_embedding['lexical_weights'], document_embeddings[i]['lexical_weights'])
        colbert_score = embedding_model.colbert_score(query_embedding['colbert_vecs'], document_embeddings[i]['colbert_vecs'])
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
def local_selector(messages:List[Message],stream=True,rag=True,course=None):
    insert_document = ""
    user_message = messages[-1].content
    if rag:
        if course == "EE 106B":
            picklefile = "recursive_seperate_none_BGE_embedding_400_106_full.pkl"
        elif course == "Public Domain Server":
            picklefile = "Berkeley.pkl"
        else:
            picklefile = "Berkeley.pkl"
        path_to_pickle = os.path.join("./app/embedding/", picklefile)
        with open(path_to_pickle, 'rb') as f:
            data_loaded = pickle.load(f)
        doc_list = data_loaded['doc_list']
        id_list = data_loaded['id_list']
        url_list = data_loaded['url_list']
        query_embed = embedding_model.encode(user_message, return_dense=True, return_sparse=True,
                                                return_colbert_vecs=True)
        if SQLDB:
            db = connect('embeddings.db')
            cur = db.cursor()
            cur.execute('Drop table IF EXISTS embeddings;')
            cur.execute('create virtual table embeddings using vss0(embedding(1024) factory="Flat,IDMap2" metric_type=INNER_PRODUCT);')
            embedding_list = data_loaded['embedding_list']
            denses = [embedding['dense_vecs'].tolist() for embedding in embedding_list]
            insert(cur, denses)
            db.commit()
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
            top_indices = [result[0] for result in results]
            top_ids = id_list[top_indices]
            distances = [result[1] for result in results]
            print("top_ids:", top_ids)
            print("distances:", distances)
            id_doc_url_dic = {id_: (doc, url) for id_, doc, url in zip(id_list, doc_list, url_list)}
            top_docs_urls = [id_doc_url_dic[top_id] for top_id in top_ids]
            top_docs, top_urls = zip(*top_docs_urls)
            print("top_docs:", top_docs, "top_urls:", top_urls)
        else:
            embedding_list = data_loaded['embedding_list']
            cosine_similarities = np.array(bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)['colbert+sparse+dense'])
            indices = np.argsort(cosine_similarities)[::-1]
            id = id_list[indices]
            docs = doc_list[indices]
            url = url_list[indices]
            print("indices:", indices)
            print("id:", id)
            print("docs:", docs)
            top_docs=docs[:3]
            distances = np.sort(cosine_similarities)[-3:][::-1]
            top_ids = id[:3]
            top_urls = url[:3]
            print("top_ids:", top_ids)
            print("distances:", distances)

        insert_document = ""
        reference = []
        n=0
        none=0
        for i in range(len(top_docs)):
            if top_urls[i]:
                reference.append(f"{top_urls[i]}")
            else:
                reference.append("")
            if distances[i] > 0.45:
                n+=1
                if top_urls[i]:
                    insert_document += f"\"\"\"Reference Number: {n}\nReference Info Path: {top_ids[i]}\nReference_Url: {top_urls[i]}\nDocument: {top_docs[i]}\"\"\"\n\n"
                else:
                    cleaned_path = clean_path(top_ids[i])
                    insert_document += f"\"\"\"Reference Number: {n}\nReference Info Path: {cleaned_path}\nReference_Url: NONE\nDocument: {top_docs[i]}\"\"\"\n\n"
            else:
                reference.append("")
                none+=1
                print(none)
        print(reference)
    if (not insert_document) or none==3:
        print("NO REFERENCES")
        user_message = f'Answer the instruction\n---\n{user_message}'
    else:
        print("INSERT DOCUMENT",insert_document)
        insert_document += f'Instruction: {user_message}'
        user_message = f"Understand the reference documents and use them to answer the instruction thoroughly, add suffiecient steps. List the references numbered, if URL does not exist then print reference info path as is do not print NONE, if url exists then print [reference Name](URL), then summarize the document in 2 sentences. Example Reference: Reference 1: Find information at (Reference Path Info). If Reference_URL is not NONE then print URL [Reference Name](URL). Then print 2 sentence summary of reference document. \n---\n{insert_document}"        # user_message = f"Understand the {n} reference documents and use it to answer the instruction. After answering the instruction, please list references. Print References numbered, if URL exists return [reference summary](URL), then return the reference and summarize the document in 2 sentences.\n---\n{insert_document}"

    print("USER MESSAGE",user_message)
    messages[-1].content = user_message

    print(messages)
    streamer_iterator=transformers.TextIteratorStreamer(auto_tokenizer, skip_prompt=True)
    t = Thread(target=prompt_generator, args=(messages,streamer_iterator,))
    t.start()
    response = streamer_iterator
    return response

def local_parser(stream):
    for chunk in stream:
        result = chunk.replace("<|eot_id|>", "")
        if result == None:
            yield ""
        else:
            yield chunk.replace("<|eot_id|>", "")

def local_formatter(messages: List[ROARChatCompletionMessage]) -> List[Message]:
    response: List[Message] = [] # type: ignore
    system_message = "You are a Teaching Assistant. You are responsible for answering questions and providing guidance to students."
    response.append(Message(role="system", content=system_message))
    for message in messages:
        response.append(Message(role=message.role, content=message.content))
    return response
