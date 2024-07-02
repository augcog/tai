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
        embedding_list = data_loaded['embedding_list']
        id_list = data_loaded['id_list']
        url_list = data_loaded['url_list']
        # time_list = data_loaded['time_list']

        query_embed = embedding_model.encode(user_message, return_dense=True, return_sparse=True,
                                                 return_colbert_vecs=True)
        # model
        # cosine_similarities = np.dot(embedding_list, query_embed)
        cosine_similarities = np.array(bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)['colbert+sparse+dense'])
        indices = np.argsort(cosine_similarities)[::-1]
        id = id_list[indices]
        docs = doc_list[indices]
        url = url_list[indices]
        # time = time_list[indices]
        top_docs=docs[:3]

        distances = np.sort(cosine_similarities)[-3:][::-1]
        top_id = id[:3]
        top_url = url[:3]
        # top_url= [f"https://www.youtube.com/watch?v={i}" for i in range(1,4)]
        # top_time = time[:3]
        insert_document = ""
        reference = []
        n=0
        for i in range(len(top_docs)):
            if top_url[i]:
                reference.append(f"{top_url[i]}")
            else:
                reference.append("")
            if distances[i] > 0.45:
                n+=1
                if top_url[i]:
                    insert_document += f"\"\"\"Reference Number: {n}\nReference: {top_id[i]}\nReference Url: {top_url[i]}\nDocument: {top_docs[i]}\"\"\"\n\n"
                else:
                    cleaned_path = clean_path(top_id[i])
                    insert_document += f"\"\"\"Reference Number: {n}\nReference: {cleaned_path}\nDocument: {top_docs[i]}\"\"\"\n\n"
                    # print("CLEANED PATH",cleaned_path)
                print(top_id[i])
        print(reference)
    if not insert_document:
        user_message = f'Answer the instruction\n---\n{user_message}'
        # insert_document+="用中文回答我的指示\n"
        # system_message="用中文回答我的指示"
        # print(chat_completion(system_message, insert_document))
    else:
        print("INSERT DOCUMENT",insert_document)
        insert_document += f'Instruction: {user_message}'
        # insert_document += "用中文回答我的指示\n"
        user_message = f"Understand the {n} reference documents and use it to answer the instruction. If there is no reference url print Reference of the document used to answer instruction. If reference url exists in the documents add at end [reference summary](URL).\n---\n{insert_document}"
        # system_message="通过阅读以下材料,用中文回答我的指示"
        # print(chat_completion(system_message, insert_document))
    print("USER MESSAGE",user_message)
    messages[-1].content = user_message

    print(messages)
    streamer_iterator=transformers.TextIteratorStreamer(auto_tokenizer, skip_prompt=True)
    t = Thread(target=prompt_generator, args=(messages,streamer_iterator,))
    t.start()
    # for i in streamer_iterator:
    #     print(i, end="")
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