from datetime import datetime
import os
import time
import numpy as np
import pickle
import openai
import cohere
import voyageai
from voyageai import get_embedding
from transformers import AutoModel, AutoTokenizer
from dotenv import load_dotenv
import torch
import torch.nn.functional as F
from torch import Tensor
from FlagEmbedding import BGEM3FlagModel
from angle_emb import AnglE, Prompts

load_dotenv()
print(os.getenv("OPENAI_API_KEY"))

# TODO TECHNIQUE
# technique = 'none'
# technique = 'seperate_paragraph'
# technique = 'bullet'
# technique = 'connected_bullet'
# technique = 'seperate_paragraph_bullet'
# technique = 'seperate_sentence'
technique = 'recursive_seperate'
# TODO METHOD
# method='to_task'
# method='to_doc'
# method='to_doc_chat_completion'
# method = 'to_task_chat_completion'
method = 'none'
# method='sum'
# TODO MODEL
# model='local'
# model = 'openai'
# model = 'openai_ada_002'
# model = 'openai_3_small'
# model = 'openai_3_large'
# model='cohere'
# model='voyage'
# model='jina'
# model='zephyr'
# model = 'SFR'
# model = 'BGE'
# model = 'e5-mistral'
# model='UAE-Large'
model='BGE'
# TODO BGE TECHNIQUE
if model=='BGE':
    # bge_technique='colbert'
    # bge_technique='sparse'
    # bge_technique='dense'
    # bge_technique='sparse+dense'
    bge_technique='sparse+dense+colbert'
    # bge_technique='sparse+e5-mistral+colbert'
def wizard_coder(history: list[dict]):
    DEFAULT_SYSTEM_PROMPT = history[0]['content']+'\n\n'
    B_INST, E_INST = "### Instruction:\n", "\n\n### Response:\n"
    messages = history.copy()
    messages_list=[DEFAULT_SYSTEM_PROMPT]
    messages_list.extend([
        f"{B_INST}{(prompt['content']).strip()}{E_INST}{(answer['content']).strip()}\n\n"
        for prompt, answer in zip(messages[1::2], messages[2::2])
    ])
    messages_list.append(f"{B_INST}{(messages[-1]['content']).strip()}{E_INST}")
    return "".join(messages_list)

def gpt(history: list[dict]):
    l=[x['content'] for x in history]
    return '\n---\n'.join(l)


def generate_log(success_retrieve, fail_retrieve,time_taken,filename=None):
    """
    Generate a logging-style output based on the success_retrieve and fail_retrieve lists.
    Write the output to a file within the "log" folder based on the current date and time.

    Parameters:
    - success_retrieve: List of successful retrievals
    - fail_retrieve: List of failed retrievals

    Returns:
    - The path to the created log file
    """

    # Calculate average
    avg = sum(int(x[1]) for x in success_retrieve) / len(success_retrieve)
    count_top_1 = sum(int(x[1]) == 0 for x in success_retrieve)
    count_top_2 = sum(int(x[1]) <=1 for x in success_retrieve)
    count_top_3 = sum(int(x[1]) <=2 for x in success_retrieve)
    count_top_5 = sum(int(x[1]) <=4 for x in success_retrieve)

    # Get the current date and time to generate a filename
    current_time = datetime.now().strftime('%m-%d_%H-%M')
    if filename is None:
        filename = f"{current_time}_log.txt"
    else:
        filename = f"{current_time}_{filename}.txt"
    folder_path = "log"

    # Create the 'log' folder if it doesn't exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Write the output to the file
    with open(os.path.join(folder_path, filename), 'w') as file:
        file.write("Time taken for embeddings: {:.2f} seconds\n".format(time_taken))
        file.write("number of success: " + str(len(success_retrieve)) + "\n")
        file.write("number of fail: " + str(len(fail_retrieve)) + "\n")
        file.write("number of top 1: " + str(count_top_1) + "\n")
        file.write("number of top 2: " + str(count_top_2) + "\n")
        file.write("number of top 3: " + str(count_top_3) + "\n")
        file.write("number of top 5: " + str(count_top_5) + "\n")
        file.write(f"Average index: {avg}\n")
        for i in fail_retrieve:
            file.write(f"id:{i[0]}\n"
                       f"question:{i[1]}\n")

    return os.path.join(folder_path, filename)

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
    # for doc_embedding, secondary_doc_embedding in zip(document_embeddings, secondary_document_embeddings):
        # Compute scores for the current document embedding against the query embedding
        if 'dense' in bge_technique:
            dense_score = query_embedding['dense_vecs'] @ document_embeddings[i]['dense_vecs'].T
        else:
            dense_score = np.dot(secondary_query_embedding, secondary_document_embeddings[i])
        # dense_score = embedding_model.dense_score(query_embedding['dense_vecs'], doc_embedding['dense_vecs'])
        sparse_score = embedding_model_add.compute_lexical_matching_score(query_embedding['lexical_weights'], document_embeddings[i]['lexical_weights'])
        colbert_score = embedding_model_add.colbert_score(query_embedding['colbert_vecs'], document_embeddings[i]['colbert_vecs'])

        # Assuming scores are returned as tensors, convert them to Python scalars
        colbert_score_val = colbert_score
        sparse_score_val = sparse_score
        dense_score_val = dense_score

        # Store the scores
        all_scores['colbert'].append(colbert_score_val)
        all_scores['sparse'].append(sparse_score_val)
        all_scores['dense'].append(dense_score_val)
        all_scores['sparse+dense'].append(
            (sparse_score_val * weights_for_different_modes[1] + dense_score_val * weights_for_different_modes[0]) /
            (weights_for_different_modes[1] + weights_for_different_modes[0])
        )
        all_scores['colbert+sparse+dense'].append(
            (colbert_score_val * weights_for_different_modes[2] + sparse_score_val * weights_for_different_modes[1] +
             dense_score_val * weights_for_different_modes[0]) / weight_sum
        )

    return all_scores

# for n in [900,800,700,600,500,400,300,200,100]:
# TODO TOKEN SIZE
for n in [400]:
    if method=='to_task':
        system_embedding_prompt = ("Given the content and the document_hierarchy_path of a document, describe the tasks you can answer based on its content.")
        system_query_prompt = 'Rephrase the provided task in your own words without changing its original meaning.'
    elif method=='to_doc':
        system_embedding_prompt = ("Summarize the content of the given document and its document_hierarchy_path. Think of the related tasks and scenarios it can help with.")
        system_query_prompt = 'Given the task, generate a document that can help you to answer this task.'
    elif method=='to_doc_chat_completion':
        system_query_prompt = 'Given an answer, find the answer in the embedding that is the closest to it.'

    human_embedding_prompt= 'document_hierarchy_path: {segment_path}\ndocument: {segment}\n'

    if model=='local'or model=='zephyr':
        openai.api_key = "empty"
        openai.api_base = "http://localhost:8000/v1"
    elif model in ['openai_ada_002','openai_3_small','openai_3_large']:
        # print(os.getenv("OPENAI_API_KEY"))
        openai.api_key = os.getenv("OPENAI_API_KEY")
        # print(openai.api_key)
    elif model=='cohere':
        co = cohere.Client(os.getenv("COHERE_API_KEY"))
    elif model=='voyage':
        voyageai.api_key = os.getenv("VOYAGE_API_KEY")
    elif model=='jina':
        jina = AutoModel.from_pretrained('jinaai/jina-embeddings-v2-base-en', trust_remote_code=True)
    elif model=='SFR':
        task = 'Given a web search query, retrieve relevant passages that answer the query'
        def last_token_pool(last_hidden_states: Tensor,
                            attention_mask: Tensor) -> Tensor:
            left_padding = (attention_mask[:, -1].sum() == attention_mask.shape[0])
            if left_padding:
                return last_hidden_states[:, -1]
            else:
                sequence_lengths = attention_mask.sum(dim=1) - 1
                batch_size = last_hidden_states.shape[0]
                return last_hidden_states[torch.arange(batch_size, device=last_hidden_states.device), sequence_lengths]


        def SFR(task_description: str, query: str) -> str:
            return f'Instruct: {task_description}\nQuery: {query}'
        # load model and tokenizer
        tokenizer = AutoTokenizer.from_pretrained('Salesforce/SFR-Embedding-Mistral')
        embedding_model = AutoModel.from_pretrained('Salesforce/SFR-Embedding-Mistral')
        tokenizer.add_eos_token = True

        # get the embeddings
        max_length = 4096
    elif model == 'e5-mistral' or 'e5-mistral' in bge_technique:
        task = 'Given a web search query, retrieve relevant passages that answer the query'
        def last_token_pool(last_hidden_states: Tensor,
                            attention_mask: Tensor) -> Tensor:
            left_padding = (attention_mask[:, -1].sum() == attention_mask.shape[0])
            if left_padding:
                return last_hidden_states[:, -1]
            else:
                sequence_lengths = attention_mask.sum(dim=1) - 1
                batch_size = last_hidden_states.shape[0]
                return last_hidden_states[torch.arange(batch_size, device=last_hidden_states.device), sequence_lengths]


        def SFR(task_description: str, query: str) -> str:
            return f'Instruct: {task_description}\nQuery: {query}'
        tokenizer = AutoTokenizer.from_pretrained('intfloat/e5-mistral-7b-instruct')
        embedding_model = AutoModel.from_pretrained('intfloat/e5-mistral-7b-instruct')
        tokenizer.add_eos_token = True

        # get the embeddings
        max_length = 4096
    elif model == 'UAE-Large':
        angle = AnglE.from_pretrained('WhereIsAI/UAE-Large-V1', pooling_strategy='cls').cuda()
        angle.set_prompt(prompt=Prompts.C)

    if model=='BGE':
        from FlagEmbedding import BGEM3FlagModel
        embedding_model_add = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)

    # RERANKER
    from FlagEmbedding import FlagReranker

    reranker = FlagReranker('BAAI/bge-reranker-large',
                            use_fp16=True)  # Setting use_fp16 to True speeds up computation with a slight performance degradation
    def chat_completion(system_message, human_message):
        system_message = system_message
        messages=[{"role": "system", "content": system_message}, {"role": "user", "content": human_message}]
        # if model=='local':
        #     prompt=wizard_coder(history)
        # elif model=='openai':
        #     prompt=gpt(history)
        # print(prompt)
        completion = openai.ChatCompletion.create(
            model='gpt-3.5-turbo', messages=messages, temperature=0
        )
        # print(completion)
        answer=completion['choices'][0]['message']["content"]

        return answer


    start=time.time()
    def remove_number(input_str):
        # Splitting the string by spaces
        parts = input_str.split(' ')

        # Removing the last part which contains the number in parentheses
        parts[-1] = parts[-1].split('(')[0]

        # Joining the parts back together
        output_str = ' '.join(parts)

        return output_str

    # Questions from pkl file
    with open("questions/zephyr_400_questions.pkl", 'rb') as f:
        questions = pickle.load(f)
    for i in questions:
        print(i)
    questions = [(remove_number(i[0]),i[1]) for i in questions]
    success_retrieve_reranker = []
    fail_retrieve_reranker = []
    success_retrieve = []
    success_page_retrieve=[]
    fail_retrieve = []
    fail_page_retrieve=[]
    success_multi_retrieve=[]
    fail_multi_retrieve=[]

    '''
    obtain from embeddings
    '''
    # n = 2300
    if technique=='recursive_seperate':

        "recursive_seperate_none_openai_embedding_1100.pkl"
        with open(f'pickle/{technique}_{method}_{model}_embedding_{n}.pkl', 'rb') as f:
            data_loaded = pickle.load(f)
        if model=='BGE' and 'e5-mistral' in bge_technique:
            with open(f'pickle/{technique}_{method}_e5-mistral_embedding_{n}.pkl', 'rb') as f:
                secondary_data_loaded = pickle.load(f)
    else:
        with open(f'pickle/{technique}_{method}_{model}_embedding.pkl', 'rb') as f:
            data_loaded = pickle.load(f)

    id_list = data_loaded['id_list']
    doc_list = data_loaded['doc_list']
    embedding_list = data_loaded['embedding_list']
    secondary_id_list = None
    secondary_doc_list = None
    secondary_embedding_list = None
    secondary_query_embed = None
    if model=='BGE' and 'e5-mistral' in bge_technique:
        secondary_id_list = secondary_data_loaded['id_list']
        secondary_doc_list = secondary_data_loaded['doc_list']
        secondary_embedding_list = secondary_data_loaded['embedding_list']




    # Define the folder name
    folder_name = "question_set"

    # Create the folder if it does not exist
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # Change the current working directory to the new folder
    os.chdir(folder_name)
    for i,(id, question) in enumerate(questions):
        if method == 'none' or method == 'sum' or method == 'connected_bullet' or method == 'to_task_chat_completion':
            history = [{"role": "user", "content": question}]
        elif method == 'to_doc_chat_completion':
            system_prompt='Generate the answer to this question'
            history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": chat_completion(system_prompt,question)}]
        else:
            history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": question}]
        # q = collection.query(query_texts=wizard_coder(history), n_results=10, include=["distances"])
        if model=='local':
            query_embed=np.array(openai.Embedding.create(model="text-embedding-ada-002", input=wizard_coder(history))['data'][0]['embedding'])
        elif (model=='openai_ada_002'):
            query_embed=np.array(openai.Embedding.create(model="text-embedding-ada-002", input=gpt(history))['data'][0]['embedding'])
        elif (model=='openai_3_small'):
            query_embed=np.array(openai.Embedding.create(model="text-embedding-3-small", input=gpt(history))['data'][0]['embedding'])
        elif (model=='openai_3_large'):
            query_embed=np.array(openai.Embedding.create(model="text-embedding-3-large", input=gpt(history))['data'][0]['embedding'])
        elif (model=='zephyr'):
            query_embed=np.array(openai.Embedding.create(model="text-embedding-ada-002", input=gpt(history))['data'][0]['embedding'])
        elif model=='cohere':
            query_embed=np.array(co.embed(texts=[question],
                                          model="embed-english-v3.0",
                                          input_type="search_query").embeddings[0])
        elif model=='voyage':
            query_embed=np.array(get_embedding(question, model="voyage-01"))
        elif model=='jina':
            query_embed=np.array(jina.encode([question])[0])
        elif model in ['SFR', 'e5-mistral'] or 'e5-mistral' in bge_technique:
            batch_dict = tokenizer([SFR(task, question)], return_tensors="pt", padding=True, truncation=True, max_length=max_length-1)
            output = embedding_model(**batch_dict)
            embed=last_token_pool(output.last_hidden_state, batch_dict['attention_mask'])
            normalized_embed = F.normalize(embed, p=2, dim=1)
            if 'e5-mistral' in bge_technique:
                secondary_query_embed = np.array(normalized_embed.detach().numpy()[0]).T
            else:
                query_embed=np.array(normalized_embed.detach().numpy()[0]).T
        elif model=='UAE-Large':
            query_embed = angle.encode({'text': question}, to_numpy=True)[0]
        if model == 'BGE':
            query_embed = embedding_model_add.encode(question, return_dense=True, return_sparse=True, return_colbert_vecs=True)
        if model == 'BGE':
            score_list = bge_compute_score(query_embed, embedding_list, [0.4, 0.2, 0.4], secondary_query_embed, secondary_embedding_list)
            if bge_technique == 'colbert':
                cosine_similarities = score_list['colbert']
            elif bge_technique == 'sparse':
                cosine_similarities = score_list['sparse']
            elif bge_technique == 'dense':
                cosine_similarities = score_list['dense']
            elif bge_technique == 'sparse+dense':
                cosine_similarities = score_list['sparse+dense']
            elif bge_technique in ['sparse+dense+colbert', 'sparse+e5-mistral+colbert']:
                cosine_similarities = score_list['colbert+sparse+dense']
            cosine_similarities = np.array(cosine_similarities)
        else:
            print(query_embed.shape)
            print(embedding_list.shape)
            # Compute cosine similarity
            cosine_similarities = np.dot(embedding_list, query_embed)



        # Get top 10 indices
        top_10_indices = np.argsort(cosine_similarities)[::-1]
        ids=id_list[top_10_indices]
        distances=cosine_similarities[top_10_indices]
        documents=doc_list[top_10_indices]



        print(question)
        question=question
        doc=documents[:3]
        doc_id=ids[:3]

        # Prepare the data to be pickled as a dictionary
        data_to_pickle = {
            'question': question,
            'doc': doc,
            'doc_id': doc_id
        }

        # Pickle the data to a file
        with open(f'data_{i}.pkl', 'wb') as file:
            pickle.dump(data_to_pickle, file)
        print(id)
        print("__________________________")
        ids= list(ids)
        seen = set()
        print(f"top3: segment")
        for i in ids[:3]:
            print(i)
        # for i in documents[:3]:
        #     print(i)

        print("=======================")
        ids_without_number = [remove_number(id) for id in ids if not (remove_number(id) in seen or seen.add(remove_number(id)))][:10]
        for i in ids_without_number:
            print(i)
        if id in ids_without_number:
            print("Success")
            k = ids_without_number.index(id)
            print("Index:", k)
            success_retrieve.append((id, k))
        else:
            print("Failed")
            fail_retrieve.append((id, question))

        # RERANKER
        ranker = []
        top_10_documents = documents[:6]
        top_10_ids = np.array(ids[:6])
        for doc in top_10_documents:
            score = reranker.compute_score([question, doc])
            print(score)
            ranker.append(score)
        sorted_indices = np.argsort(ranker)[::-1]
        reranked_10_docs = top_10_documents[sorted_indices]
        reranked_10_ids = top_10_ids[sorted_indices]
        reranked_10_ids = list(reranked_10_ids)
        seen = set()
        sorted_ids_without_number = [remove_number(id) for id in reranked_10_ids if not (remove_number(id) in seen or seen.add(remove_number(id)))]
        for i in sorted_ids_without_number:
            print(i)
        if id in sorted_ids_without_number:
            print("Success")
            k = sorted_ids_without_number.index(id)
            print("Reranker Index:", k)
            success_retrieve_reranker.append((id, k))
        else:
            print("Failed")
            fail_retrieve_reranker.append((id, question))
        page_id = id.split(' > (h1)')[0]
        if sum(page_id in i for i in ids_without_number) > 0:
            print("Success in page")
            k = [i for i, s in enumerate(ids_without_number) if page_id in s][0]
            print("Index:", k)
            success_page_retrieve.append((id, k))
        else:
            print("Failed in page")
            fail_page_retrieve.append((id, question))

        id_page1=ids_without_number[0].split(' > (h1)')[0]
        id_page2=ids_without_number[1].split(' > (h1)')[0]
        print("id_page1: ",id_page1)
        print("id_page2: ",id_page2)
        idl=list(id_list)
        page_index=[i for i, s in enumerate(idl) if id_page1 in s or id_page2 in s]
        embedding_page_list=embedding_list[page_index]
        doc_page_list=doc_list[page_index]
        id_page_list=id_list[page_index]
        # Compute cosine similarity
        if model == 'BGE':
            score_list = bge_compute_score(query_embed, embedding_page_list, [0.4, 0.2, 0.4], secondary_query_embed, secondary_embedding_list)
            if bge_technique == 'colbert':
                cosine_similarities = score_list['colbert']
            elif bge_technique == 'sparse':
                cosine_similarities = score_list['sparse']
            elif bge_technique == 'dense':
                cosine_similarities = score_list['dense']
            elif bge_technique == 'sparse+dense':
                cosine_similarities = score_list['sparse+dense']
            elif bge_technique in ['sparse+dense+colbert', 'sparse+e5-mistral+colbert']:
                cosine_similarities = score_list['colbert+sparse+dense']
            cosine_similarities = np.array(cosine_similarities)
        else:
            print(query_embed.shape)
            print(embedding_list.shape)
            # Compute cosine similarity
            cosine_similarities = np.dot(embedding_list, query_embed)
        # cosine_similarities = np.dot(embedding_page_list, query_embed)  # Dot product since vectors are normalized
        # Get top 10 indices
        top_10_indices = np.argsort(cosine_similarities)[::-1]
        ids = id_page_list[top_10_indices]
        distances = cosine_similarities[top_10_indices]
        documents = doc_page_list[top_10_indices]

        ids = list(ids)
        seen = set()
        ids_without_number = [remove_number(id) for id in ids if not (remove_number(id) in seen or seen.add(remove_number(id)))][:10]
        print("id: ", ids_without_number)

        if id in ids_without_number:
            print("Success in multi step")
            k = ids_without_number.index(id)
            print("Index:", k)
            success_multi_retrieve.append((id, k))
        else:
            print("Failed in multi step")
            fail_multi_retrieve.append((id, question))
    os.chdir('..')
    query_time = time.time() - start
    query_time = time.time() - start
    if model == 'BGE':
        log_path_seg = generate_log(success_retrieve, fail_retrieve, query_time,
                                    filename=f"{technique}_{method}_{model}_{bge_technique}_{n}_seg")
        log_path_reranker = generate_log(success_retrieve_reranker, fail_retrieve_reranker, query_time,
                                            filename=f"{technique}_{method}_{model}_{bge_technique}_{n}_reranker")
        log_path_page = generate_log(success_page_retrieve, fail_page_retrieve, query_time,
                                     filename=f"{technique}_{method}_{model}_{bge_technique}_{n}_page")
        log_path_multi = generate_log(success_multi_retrieve, fail_multi_retrieve, query_time,
                                      filename=f"{technique}_{method}_{model}_{bge_technique}_{n}_multi")
    elif technique == 'recursive_seperate':
        log_path_seg = generate_log(success_retrieve, fail_retrieve, query_time,
                                    filename=f"{technique}_{method}_{model}_{n}_seg")
        log_path_reranker = generate_log(success_retrieve_reranker, fail_retrieve_reranker, query_time,
                                        filename=f"{technique}_{method}_{model}_{n}_reranker")
        log_path_page = generate_log(success_page_retrieve, fail_page_retrieve, query_time,
                                     filename=f"{technique}_{method}_{model}_{n}_page")
        log_path_multi = generate_log(success_multi_retrieve, fail_multi_retrieve, query_time,
                                      filename=f"{technique}_{method}_{model}_{n}_multi")
    else:
        log_path_seg = generate_log(success_retrieve, fail_retrieve, query_time,
                                    filename=f"{technique}_{method}_{model}_seg")
        log_path_reranker = generate_log(success_retrieve_reranker, fail_retrieve_reranker, query_time,
                                        filename=f"{technique}_{method}_{model}_reranker")
        log_path_page = generate_log(success_page_retrieve, fail_page_retrieve, query_time,
                                     filename=f"{technique}_{method}_{model}_page")
        log_path_multi = generate_log(success_multi_retrieve, fail_multi_retrieve, query_time,
                                      filename=f"{technique}_{method}_{model}_multi")

    # Example of how you might print or use the log paths
    print(f"Segment log saved to: {log_path_seg}")
    print(f"Page log saved to: {log_path_page}")
    print(f"Multi-step log saved to: {log_path_multi}")
    print('query time:',query_time)




