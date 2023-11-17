# text=''
# embedding=

#for loop
#random choose doc (seed or pick)
#generate question

#apply different technique
#1. multi step


#do query
#evaluate result


import os
import time
import chromadb
from chromadb.utils import embedding_functions
import pickle
import openai



# history=[]
# history.append({"role": "system", "content": client.starting_prompt})
# history.append({"role": "user", "content": message})

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

# system_embedding_prompt = ("1) You will be given information of a website. \n"
#                            "2) Page_path is the subdirectory of the website link\n"
#                            "Example: "
#                            "3) Section_path is \n"
#                            "4) Paraphrase the segment content with description\n"
#                            "5) You will be given text's in Markdown format\n")
# # folder_tree, folder_path, segment_tree, segment_path, segment
# human_embedding_prompt= 'Page_path: {folder_path}\nSection_path: {segment_path}\nSection: {segment}'
# system_query_prompt= 'you need to give description of a section of the document that is most related to the provided task or scenario for a person who have not read the original document to start with.'





chroma_client = chromadb.Client()

os.environ["OPENAI_API_BASE"] = "http://localhost:8000/v1"
os.environ["OPENAI_API_KEY"] = "empty"
openai_ef = embedding_functions.OpenAIEmbeddingFunction(
                api_key="empty",
                model_name="text-embedding-ada-002",
                api_base="http://localhost:8000/v1"
            )
collection = chroma_client.create_collection(name="abstracts", embedding_function=openai_ef)
sp = system_embedding_prompt
hp = human_embedding_prompt.format(folder_path=folder_path, segment_path=segment_path, segment=segment)
history = [{"role": "system", "content": sp}, {"role": "user", "content": hp}]
input = wizard_coder(history)
# print(segment)
embedding.append(openai.Embedding.create(model="text-embedding-ada-002", input=input)['data'][0]['embedding'])
