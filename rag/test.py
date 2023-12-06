import os
import pickle

picklefile = "recursive_seperate_none_openai_embedding_400_textbook.pkl"


path_to_pickle = os.path.join("/home/bot/roarai/rag/pickle/", picklefile)
with open(path_to_pickle, 'rb') as f:
    data_loaded = pickle.load(f)
doc_list = data_loaded['doc_list']
embedding_list = data_loaded['embedding_list']
id_list = data_loaded['id_list']
for i in id_list:
    print(i)