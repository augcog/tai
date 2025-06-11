import os
import pickle
import numpy as np
from FlagEmbedding import BGEM3FlagModel


def generate_test_pickle(filepath: str):
    """
    Generates a test pickle file using real embeddings computed from document texts.

    The pickle file contains:
      - doc_list: list of document contents (strings)
      - id_list: list of document IDs (strings)
      - url_list: list of URLs corresponding to the documents (strings)
      - embedding_list: list of embeddings; each is a dict with keys:
            'dense_vecs', 'lexical_weights', and 'colbert_vecs'

    This approach uses the actual embedding model (BGEM3FlagModel) to compute embeddings,
    ensuring that dot products (or other similarity measures) naturally reflect textual similarity.
    """
    # Define your documents and associated metadata.
    doc_list = [
        "Document 1: Introduction to testing. Dummy prompt text for reference.",
        "Document 2: Advanced testing methods. More dummy text here.",
        "Document 3: Test-driven development practices. Even more dummy text.",
        "Document 4: UC Berkeley TAI (Teaching Assistant Intelligence) project is a great research project :)!"
    ]
    id_list = ["doc1", "doc2", "doc3", "doc4"]
    url_list = [
        "https://www.google.com/",
        "https://www.berkeley.edu/",
        "",  # Document 3 has no URL.
        "https://github.com/augcog/tai"
    ]

    # Initialize the embedding model.
    embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)

    embedding_list = []
    # For each document, compute its embedding using the actual algorithm.
    for doc in doc_list:
        embedding = embedding_model.encode(
            doc,
            return_dense=True,
            return_sparse=True,
            return_colbert_vecs=True
        )
        embedding_list.append(embedding)

    # Pack the data into a dictionary.
    data = {
        'doc_list': np.array(doc_list),
        'id_list': np.array(id_list),
        'url_list': np.array(url_list),
        'embedding_list': embedding_list
    }

    # Write the data to a pickle file.
    with open(filepath, 'wb') as f:
        pickle.dump(data, f)
    print(f"Test pickle file generated at: {filepath}")


if __name__ == "__main__":
    # Generate the test pickle file in the current directory.
    output_dir = os.getcwd()
    pickle_filepath = os.path.join(output_dir, "Berkeley.pkl")
    generate_test_pickle(pickle_filepath)
