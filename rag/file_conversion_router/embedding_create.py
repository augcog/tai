from datetime import datetime
import os
import time
import numpy as np
import pickle
import openai
from tqdm import tqdm
import cohere
import voyageai
from voyageai import get_embedding
from transformers import AutoModel, AutoTokenizer
import string
import tiktoken
from dotenv import load_dotenv
import torch
import torch.nn.functional as F
from torch import Tensor
from angle_emb import AnglE, Prompts

load_dotenv()


def string_subtraction(main_string, sub_string):
    return main_string.replace(sub_string, '', 1)  # The '1' ensures only the first occurrence is removed

def traverse_files(path, start_folder_name, url_list, id_list, doc_list):
    results = []
    # Check if the provided path exists
    if not os.path.exists(path):
        raise ValueError(f"The provided path '{path}' does not exist.")
    folder_tree = f"{start_folder_name} (h1)\n"
    for root, dir, files in os.walk(path):
        for file in files:
            if file.endswith('.pkl'):
                path_list = [start_folder_name] + string_subtraction(root, path).split('/')[1:]
                line = ((len(path_list) - 1) * "--" + path_list[-1] + f" (L{len(path_list)})")
                folder_tree += f"{line}\n"

    for root, dir, files in os.walk(path):
        for file in files:
            if file.endswith('.pkl'):
                # file path
                file_path = os.path.join(root, file)
                path_list = [start_folder_name] + string_subtraction(root, path).split('/')[1:]
                with open(file_path, 'rb') as pkl_file:
                    print(file_path)
                    chunks = pickle.load(pkl_file)
                for chunk in chunks:
                    folder_path = ' > '.join(f"{item} (Level{i + 1})" for i, item in enumerate(path_list))
                    page_path = chunk.titles
                    id = folder_path + ' > ' + page_path
                    id_list.append(id)
                    doc_list.append(chunk.content)
                    print(chunk.chunk_url)
                    url = "URLs:\n" + "\n".join(chunk.chunk_url)
                    url_list.append(url)
'''
Traverse through files
'''
# Process each page
# TODO PROCESS DOCUMENTS
# docs = traverse_files("Moveit_pkl", "Moveit")
#
# # TODO TECHNIQUE
# technique = 'recursive_seperate'
# # TODO METHOD
# method='none'
# # TODO MODEL
# model='local'
# model='openai'
# model = 'openai_ada_002'
# model = 'openai_3_small'
# model = 'openai_3_large'
# model='cohere'
# model='jina'
# model='zephyr'
# model='voyage'
# model='SFR'
# model='e5-mistral'
# model='UAE-Large'
model='BGE'
# model='GRITLM'

embedding_name = 'embeddings'
folder_name = "pickle"


def embedding_create(markdown_path,name, embedding_name, folder_name, model):
    '''
    Traverse through files
    '''
    id_list = []
    doc_list = []
    embedding_list = []
    url_list = []
    time_list = []
    fail = []
    start = time.time()
    # Process each page
    # TODO PROCESS DOCUMENTS
    docs = traverse_files(markdown_path, name, url_list, id_list, doc_list)

    if model == 'local' or model == 'zephyr':
        openai.api_key = "empty"
        openai.api_base = "http://localhost:8000/v1"

        def gpt(history: list[dict]):
            l = [x['content'] for x in history]
            return '\n---\n'.join(l)
    elif model in ['openai_ada_002', 'openai_3_small', 'openai_3_large']:
        openai.api_key = os.getenv("OPENAI_API_KEY")

        def gpt(history: list[dict]):
            l = [x['content'] for x in history]
            return '\n---\n'.join(l)
    elif model=='cohere':
        co = cohere.Client(os.getenv("COHERE_API_KEY"))
    elif model=='voyage':
        voyageai.api_key = os.getenv("VOYAGE_API_KEY")
    elif model=='jina':
        jina = AutoModel.from_pretrained('jinaai/jina-embeddings-v2-base-en', trust_remote_code=True)
    elif model=='SFR':
        def last_token_pool(last_hidden_states: Tensor,
                            attention_mask: Tensor) -> Tensor:
            left_padding = (attention_mask[:, -1].sum() == attention_mask.shape[0])
            if left_padding:
                return last_hidden_states[:, -1]
            else:
                sequence_lengths = attention_mask.sum(dim=1) - 1
                batch_size = last_hidden_states.shape[0]
                return last_hidden_states[torch.arange(batch_size, device=last_hidden_states.device), sequence_lengths]
        tokenizer = AutoTokenizer.from_pretrained('Salesforce/SFR-Embedding-Mistral')
        embedding_model = AutoModel.from_pretrained('Salesforce/SFR-Embedding-Mistral')
        tokenizer.add_eos_token = True

        # get the embeddings
        max_length = 4096
    elif model=='e5-mistral':
        def last_token_pool(last_hidden_states: Tensor,
                            attention_mask: Tensor) -> Tensor:
            left_padding = (attention_mask[:, -1].sum() == attention_mask.shape[0])
            if left_padding:
                return last_hidden_states[:, -1]
            else:
                sequence_lengths = attention_mask.sum(dim=1) - 1
                batch_size = last_hidden_states.shape[0]
                return last_hidden_states[torch.arange(batch_size, device=last_hidden_states.device), sequence_lengths]
        tokenizer = AutoTokenizer.from_pretrained('intfloat/e5-mistral-7b-instruct')
        embedding_model = AutoModel.from_pretrained('intfloat/e5-mistral-7b-instruct')
        tokenizer.add_eos_token = True

        # get the embeddings
        max_length = 4096
    elif model=='UAE-Large':
        angle = AnglE.from_pretrained('WhereIsAI/UAE-Large-V1', pooling_strategy='cls').cuda()
        angle.set_prompt(prompt=Prompts.C)
    elif model=='BGE':
        from FlagEmbedding import BGEM3FlagModel
        embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)
    elif model=='GRITLM':
        from gritlm import GritLM
        embedding_model = GritLM("GritLM/GritLM-7B", torch_dtype="auto")
    for i in range(len(doc_list)):
        human_embedding_prompt= 'document_hierarchy_path: {segment_path}\ndocument: {segment}\n'
        hp = human_embedding_prompt.format(segment=doc_list[i], segment_path=id_list[i])
        try:
            history = [{"role": "user", "content": hp.strip()}]
            if model == 'local':
                embedding_list.append(openai.Embedding.create(model="text-embedding-ada-002", input=hp.strip())['data'][0]['embedding'])
            elif model == 'zephyr':
                embedding_list.append(openai.Embedding.create(model="text-embedding-ada-002", input=gpt(history))['data'][0]['embedding'])
            elif model == 'openai_ada_002':
                embedding_list.append(openai.Embedding.create(model="text-embedding-ada-002", input=gpt(history))['data'][0]['embedding'])
            elif model == 'openai_3_small':
                embedding_list.append(openai.Embedding.create(model="text-embedding-3-small", input=gpt(history))['data'][0]['embedding'])
            elif model == 'openai_3_large':
                embedding_list.append(openai.Embedding.create(model="text-embedding-3-large", input=gpt(history))['data'][0]['embedding'])
            elif model == 'cohere':
                embedding_list.extend(co.embed(texts=[hp],
                                 model="embed-english-v3.0",
                                 input_type="search_document").embeddings)
            elif model == 'voyage':
                time.sleep(1)
                embedding_list.append(get_embedding(hp, model="voyage-01"))
            elif model == 'jina':
                embedding_list.append(jina.encode([hp])[0])
            elif model in ['SFR','e5-mistral']:
                batch_dict = tokenizer([hp], max_length=max_length - 1, padding=True, truncation=True, return_tensors="pt")
                output = embedding_model(**batch_dict)
                embed=last_token_pool(output.last_hidden_state, batch_dict['attention_mask'])
                normalized_embedding= F.normalize(embed, p=2, dim=-1)
                embedding_list.extend(normalized_embedding.tolist())
            elif model == 'UAE-Large':
                embedding_list.extend(angle.encode({'text': hp},to_numpy=True))
            elif model == 'BGE':
                # print(embedding_model.encode(hp, return_dense=True, return_sparse=True, return_colbert_vecs=True))
                embedding_list.append(embedding_model.encode(hp, return_dense=True, return_sparse=True, return_colbert_vecs=True))


        except openai.error.APIError as e:
            print(f"Embedding error: {e}")
            fail.append(id_list[i])
    #         count += 1
    #     id_list.extend(ids)
    #     embedding_list.extend(embedding)
    id_list=np.array(id_list)
    doc_list=np.array(doc_list)
    embedding_list=np.array(embedding_list)
    url_list=np.array(url_list)
    time_list=np.array(time_list)
    print('create time:',time.time()-start)

    # Store the variables in a dictionary
    data_to_store = {
        'id_list': id_list,
        'doc_list': doc_list,
        'embedding_list': embedding_list,
        'url_list': url_list,
        'time_list': time_list
    }
    # Create the folder if it does not exist
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # Open a file in binary write mode and store the data using pickle

    with open(f'{folder_name}/{embedding_name}.pkl', 'wb') as f:
        pickle.dump(data_to_store, f)

    for i in fail:
        print("Failed Embeddings: ", i)


if __name__ == "__main__":
    embedding_create("./../../../cs61a_chunk", "about", "cs61a", "pickle", "BGE")
