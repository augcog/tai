import os
import pickle
import numpy as np
from dotenv import load_dotenv
from transformers import AutoTokenizer, pipeline
import openai
import pandas as pd
from typing import List

from wandb.wandb_torch import torch

from rag.Testing.retrieval_test import bge_compute_score
from rag.embedding_create import embedding_model, embedding_list, id_list, doc_list

# Load environment variables
load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")
huggingface_token = os.getenv("HUGGINGFACE_TOKEN")

# Load models and tokenizers
model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
auto_tokenizer = AutoTokenizer.from_pretrained(model_id, use_auth_token=huggingface_token)
pipeline = pipeline(
    "text-generation",
    model=model_id,
    tokenizer=auto_tokenizer,
    device="cuda",
    model_kwargs={"torch_dtype": torch.bfloat16}
)

# Load data function
def load_data(directory):
    data = []
    for filename in os.listdir(directory):
        if filename.endswith('.pkl'):
            with open(os.path.join(directory, filename), 'rb') as file:
                data.append(pickle.load(file))
    return data

data = load_data('path_to_your_data')

# Retrieval and generation functions
def retrieve_documents(query, top_n=3):
    query_embed = embedding_model.encode(query, return_dense=True, return_sparse=True, return_colbert_vecs=True)
    cosine_similarities = np.array(bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)['colbert+sparse+dense'])
    top_indices = np.argsort(cosine_similarities)[::-1][:top_n]
    return [doc_list[i] for i in top_indices], [id_list[i] for i in top_indices]

def generate_answer(context, question):
    input_text = f"Context: {context}\nQuestion: {question}"
    response = pipeline(input_text, max_new_tokens=150)
    return response[0]['generated_text']

# Evaluation function
def evaluate_answers(generated_answers, reference_answers):
    # Implement your evaluation metrics here
    pass

# Process and save results
results = []
for entry in data:
    question = entry['question']
    top_docs, doc_ids = retrieve_documents(question)
    context = " ".join(top_docs)
    generated_answer = generate_answer(context, question)
    results.append({
        "question": question,
        "generated_answer": generated_answer,
        "context": context,
        "reference_answers": entry.get('reference_answers', [])
    })

# Save results to a CSV file for further analysis
results_df = pd.DataFrame(results)
results_df.to_csv('augmented_generation_test_results.csv', index=False)
