from src import personas
from asgiref.sync import sync_to_async
import os
import pickle
import openai
import numpy as np
from dotenv import load_dotenv
from FlagEmbedding import BGEM3FlagModel
import time
# lode embedding model
embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)

load_dotenv()
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

def llama_v2_prompt(
        messages: list[dict], system_prompt: str = None
):
    B_INST, E_INST = "[INST]", "[/INST]"
    B_SYS, E_SYS = "<<SYS>>\n", "\n<</SYS>>\n\n"
    BOS, EOS = "<s>", "</s>\\"
    DEFAULT_SYSTEM_PROMPT = system_prompt
    messages=messages.copy()
    messages[0]['data']['content'] = B_SYS + DEFAULT_SYSTEM_PROMPT + E_SYS + messages[0]['data']["content"]
    messages_list = [
        f"{BOS}{B_INST} {(prompt['data']['content']).strip()} {E_INST} {(answer['data']['content']).strip()} {EOS}\n"
        for prompt, answer in zip(messages[::2], messages[1::2])
    ]
    messages_list.append(f"{BOS}{B_INST} {(messages[-1]['data']['content']).strip()} {E_INST}")

    return "".join(messages_list)

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


async def official_handle_response(message, client) -> str:
    return await sync_to_async(client.chatbot.ask)(message)


def gpt(history: list[dict]):
    l = [x['content'] for x in history]
    return '\n---\n'.join(l)

def chat_completion(system_message, human_message, model='zephyr'):
    history = [{"role": "system", "content": system_message}, {"role": "user", "content": human_message}]


    if model=='openai':
        completion=openai.ChatCompletion.create(model='gpt-3.5-turbo',messages=history, temperature=0, max_tokens=500)
        answer = completion['choices'][0]['message']["content"]
    elif model=='local':
        prompt = wizard_coder(history)
        # print(type(prompt))
        completion=openai.Completion.create(model='gpt-3.5-turbo', prompt=prompt, temperature=0, max_tokens=500)
        answer = completion['choices'][0]['text']
    elif model=='zephyr':
        completion = openai.ChatCompletion.create(model='gpt-3.5-turbo', messages=history, temperature=0,
                                                  max_tokens=500)
        answer = completion['choices'][0]['message']["content"]
    return answer
async def local_handle_response(message, client,user,stream=False,rag=False):
    # history = await client.get_chat_history(user)
    insert_document = ""
    if rag:
        picklefile = "recursive_seperate_none_BGE_embedding_400_106_full.pkl"
        path_to_pickle = os.path.join("/home/bot/roarai/rag/pickle/", picklefile)
        with open(path_to_pickle, 'rb') as f:
            data_loaded = pickle.load(f)
        doc_list = data_loaded['doc_list']
        embedding_list = data_loaded['embedding_list']
        id_list = data_loaded['id_list']


        history = [{"role": "user", "content": message}]
        # # OPENAI
        # openai.api_key = os.getenv("OPENAI_API_KEY")
        # openai.api_base = "https://api.openai.com/v1"
        query_embed = embedding_model.encode(message, return_dense=True, return_sparse=True,
                                                 return_colbert_vecs=True)
        # query_embed = np.array(openai.Embedding.create(model="text-embedding-ada-002", input=gpt(history))['data'][0]['embedding'])
        # model
        openai.api_key = "empty"
        openai.api_base = "http://localhost:8000/v1"
        # cosine_similarities = np.dot(embedding_list, query_embed)
        cosine_similarities = np.array(bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)['colbert+sparse+dense'])
        indices = np.argsort(cosine_similarities)[::-1]
        id = id_list[indices]
        docs = doc_list[indices]
        top_docs=docs[:3]

        distances = np.sort(cosine_similarities)[-3:][::-1]
        top_id = id[:3]
        print(distances)
        insert_document = ""
        for i in range(len(top_docs)):
            if distances[i] > 0.45:
                insert_document += f"\"\"\"{top_id[i]}\n{top_docs[i]}\"\"\"\n"
                print(top_id[i])
        # print(insert_document)
        count = 0
        # start_time = time.time()
        # for docu, id in zip(top_docs, top_id):
        #     # system_message = 'Review the article within the triple quotes. Determine if it can help answer the given instruction. At the end, say "True!" if the article helps, or "False!" if it does not'
        #     # system_message = 'Review the article within the triple quotes. Determine if it provides relevant information, insights, or context that could be useful, either directly or indirectly, in addressing the given instruction. At the end, say "True!" if the article helps, or "False!" if it does not'
        #     docs = f"\"\"\"{id}\n{docu}\"\"\"\n"
        #     system_message='''Review the article within the triple quotes. Determine if its context can help answer the given instruction. Either directly or indirectly are acceptable. At the end, say "True!" if the article helps, or "False!" if it does not.
        #     At the end, say "True!" if the article helps, or "False!" if it does not.
        #     At the end, say "True!" if the article helps, or "False!" if it does not.
        #     At the end, say "True!" if the article helps, or "False!" if it does not.
        #     At the end, say "True!" if the article helps, or "False!" if it does not.
        #
        #     '''
        #     question = f"Instruction:{message}\n"
        #     human_message = (docs + question)
        #     # print(human_message)
        #     # print(human_message)
        #     response = chat_completion(system_message, human_message).replace("\n\n\n", '')
        #     print("Response")
        #     print(response)
        #     print("Document")
        #     print(docu)
        #     print("ID")
        #     print(id)
        #     print('--------------------------------------------------------------------------------')
        #     if "true!" in response.lower() or "yes" in response.lower():
        #         count += 1
        #         insert_document += docs
        # print(f"Number of True: {count}")
        # print(f"Document check time: {time.time() - start_time}")
    #     # message = "question: " + message + "\n---\n" + insert_document
    #     # message = "question: " + message
    #     document_no_space=insert_document.replace("\n", ' ')
    #     message = (f"Is the question:({message.strip()}) related to document:({document_no_space})?\n"
    #                f"Just give me an Answer yes or no.\n")
    #     print(message)
    # history=None
    if not insert_document:
        insert_document = f'{message}'
        # insert_document+="用中文回答我的指示\n"
        system_message = "Answer the instruction"
        # system_message="用中文回答我的指示"
        # print(chat_completion(system_message, insert_document))
    else:
        insert_document += f'Instruction: {message}'
        # insert_document += "用中文回答我的指示\n"
        system_message = "Understand the documents and use it to answer the instruction."
        # system_message="通过阅读以下材料,用中文回答我的指示"
        # print(chat_completion(system_message, insert_document))
    history=[]
    # if history is None:
    #     history=[]
    # print(token_size(insert_document))
    if not history:
        history.append({"role": "system", "content": system_message})
    history.append({"role": "user", "content": insert_document})

    # prompt=wizard_coder(history)

    if not stream:
        response= await openai.ChatCompletion.acreate(model=client.openAI_gpt_engine, messages=history, temperature=0.3,max_tokens=1000)
        history.append(response['choices'][0]['message'].replace('\n\n\n',''))
        await client.set_chat_history(user,history)
        return response['choices'][0]['message']['content'].replace('\n\n\n','')

    else:
        response= await openai.ChatCompletion.acreate(model=client.openAI_gpt_engine, messages=history, temperature=0.3,max_tokens=1000,stream=True)
        return response,history


# prompt engineering
async def switch_persona(persona, client) -> None:
    if client.chat_model == "LOCAL":
        client.chatbot.reset_chat()
        client.chatbot.ask(personas.PERSONAS.get(persona))
    elif client.chat_model == "OFFICIAL":
        client.chatbot = client.get_chatbot_model(prompt=personas.PERSONAS.get(persona))
