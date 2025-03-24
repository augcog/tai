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
from dotenv import load_dotenv
import torch
import torch.nn.functional as F
from torch import Tensor
from angle_emb import AnglE, Prompts

from rag.file_conversion_router.utils.logger import content_logger

load_dotenv()


def string_subtraction(main_string, sub_string):
    return main_string.replace(sub_string, '', 1)  # The '1' ensures only the first occurrence is removed


def traverse_files(path, start_folder_name, url_list, id_list, doc_list, file_paths_list):
    results = []
    # Check if the provided path exists
    if not os.path.exists(path):
        raise ValueError(f"The provided path '{path}' does not exist.")
    folder_tree = f"{start_folder_name} (h1)\n"

    path = os.path.abspath(path)

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

                # Find associated file (pdf, video, or md) in the same directory
                base_name = os.path.splitext(file)[0]
                pdf_path = os.path.join(root, f"{base_name}.pdf")
                video_extensions = ['.mp4', '.avi', '.mov', '.mkv', '.webm']
                video_path = None
                for ext in video_extensions:
                    potential_video = os.path.join(root, f"{base_name}{ext}")
                    if os.path.exists(potential_video):
                        video_path = potential_video
                        break
                md_path = os.path.join(root, f"{base_name}.md")

                # Determine which file path to use based on priority
                associated_file_path = None
                if os.path.exists(pdf_path):
                    associated_file_path = pdf_path
                elif video_path and os.path.exists(video_path):
                    associated_file_path = video_path
                elif os.path.exists(md_path):
                    associated_file_path = md_path

                # Convert absolute path to relative path including the root folder name
                if associated_file_path:
                    # Get path relative to the parent directory of the root folder
                    parent_dir = os.path.dirname(path)
                    relative_file_path = os.path.relpath(associated_file_path, parent_dir)
                else:
                    relative_file_path = None

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
                    file_paths_list.append(relative_file_path)  # Add the relative file path with root folder

    return url_list, id_list, doc_list, file_paths_list


def embedding_create(markdown_path, name, embedding_name, folder_name, model):
    '''
    Traverse through files
    '''
    id_list = []
    doc_list = []
    embedding_list = []
    url_list = []
    time_list = []
    fail = []
    file_paths_list = []
    start = time.time()
    # Process each page
    url_list, id_list, doc_list, file_paths_list = traverse_files(markdown_path, name, url_list, id_list, doc_list,
                                                                  file_paths_list)
    if model == 'BGE':
        from FlagEmbedding import BGEM3FlagModel
        embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)
    for i in tqdm(range(len(doc_list))):
        human_embedding_prompt = 'document_hierarchy_path: {segment_path}\ndocument: {segment}\n'
        hp = human_embedding_prompt.format(segment=doc_list[i], segment_path=id_list[i])

        history = [{"role": "user", "content": hp.strip()}]
        if model == 'BGE':
            # print(embedding_model.encode(hp, return_dense=True, return_sparse=True, return_colbert_vecs=True))
            embedding_list.append(
                embedding_model.encode(hp, return_dense=True, return_sparse=True, return_colbert_vecs=True))

    id_list = np.array(id_list)
    doc_list = np.array(doc_list)
    embedding_list = np.array(embedding_list)
    url_list = np.array(url_list)
    time_list = np.array(time_list)
    file_paths_list = np.array(file_paths_list)
    print('create time:', time.time() - start)

    # Store the variables in a dictionary
    data_to_store = {
        'id_list': id_list,
        'doc_list': doc_list,
        'embedding_list': embedding_list,
        'url_list': url_list,
        'file_paths_list': file_paths_list,
        'time_list': time_list
    }

    validate_data(data_to_store)

    # Create the folder if it does not exist
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # Open a file in binary write mode and store the data using pickle
    with open(f'{folder_name}/{embedding_name}.pkl', 'wb') as f:
        pickle.dump(data_to_store, f)

    for i in fail:
        print("Failed Embeddings: ", i)


def validate_data(data):
    """
    Check if the content of the data is valid.
    Args:
        data (dict): The data dictionary to check.
    """

    content_logger.info("Validating embedding data...")
    lengths = []
    for key, value in data.items():
        if len(value) == 0:
            content_logger.error(f"{key} is empty.")
        if not isinstance(value, np.ndarray):
            content_logger.error(f"{key} is not a numpy array.")
        lengths.append(len(value))

    if len(set(lengths)) > 1:
        content_logger.error("Lists are of inconsistent lengths.")

    content_logger.info("Data validation complete.")


if __name__ == "__main__":
    embedding_create("/Users/lihaichao/Documents/TAI/tai/rag/file_conversion_router/services/output_files", "about", "cs61a", "pickle", "BGE")