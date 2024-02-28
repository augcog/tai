# from FlagEmbedding import BGEM3FlagModel
#
# model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True) # Setting use_fp16 to True speeds up computation with a slight performance degradation
#
# sentences_1 = ["What is BGE M3?", "Defination of BM25"]
# sentences_2 = ["BGE M3 is an embedding model supporting dense retrieval, lexical matching and multi-vector interaction.",
#                "BM25 is a bag-of-words retrieval function that ranks a set of documents based on the query terms appearing in each document"]
#
# embeddings_1 = model.encode(sentences_1,
#                             batch_size=12,
#                             max_length=8192, # If you don't need such a long length, you can set a smaller value to speed up the encoding process.
#                             return_dense=True,
#                             return_sparse=True,
#                             return_colbert_vecs=True
#
#                             )
# print(embeddings_1)
# embeddings_1 = embeddings_1['dense_vecs']
# embeddings_2 = model.encode(sentences_2)['dense_vecs']
# similarity = embeddings_1 @ embeddings_2.T
# print(similarity)
# [[0.6265, 0.3477], [0.3499, 0.678 ]]

from FlagEmbedding import BGEM3FlagModel

model = BGEM3FlagModel('BAAI/bge-m3',  use_fp16=True)

sentences_1 = ["What is BGE M3?", "Defination of BM25"]
sentences_2 = ["BGE M3 is an embedding model supporting dense retrieval, lexical matching and multi-vector interaction.",
               "BM25 is a bag-of-words retrieval function that ranks a set of documents based on the query terms appearing in each document"]

sentence_pairs = [[i,j] for i in sentences_1 for j in sentences_2]

print(model.compute_score(sentence_pairs,
                          max_passage_length=128, # a smaller max length leads to a lower latency
                          weights_for_different_modes=[0.4, 0.2, 0.4])) # weights_for_different_modes(w) is used to do weighted sum: w[0]*dense_score + w[1]*sparse_score + w[2]*colbert_score

# {
#   'colbert': [0.7796499729156494, 0.4621465802192688, 0.4523794651031494, 0.7898575067520142],
#   'sparse': [0.195556640625, 0.00879669189453125, 0.0, 0.1802978515625],
#   'dense': [0.6259765625, 0.347412109375, 0.349853515625, 0.67822265625],
#   'sparse+dense': [0.482503205537796, 0.23454029858112335, 0.2332356721162796, 0.5122477412223816],
#   'colbert+sparse+dense': [0.6013619303703308, 0.3255828022956848, 0.32089319825172424, 0.6232916116714478]
# }


