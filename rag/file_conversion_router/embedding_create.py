from datetime import datetime
import os
import time
from datetime import datetime

import numpy as np
import pickle
from dotenv import load_dotenv
from file_conversion_router.utils.logger import content_logger
from tqdm import tqdm
from FlagEmbedding import BGEM3FlagModel

load_dotenv()


def string_subtraction(main_string, sub_string):
    return main_string.replace(
        sub_string, "", 1
    )  # The '1' ensures only the first occurrence is removed


def traverse_files(
        path,
        start_folder_name
):
    url_list = []
    id_list = []
    doc_list = []
    file_paths_list = []
    topic_path_list = []
    index_list = []
    # Check if the provided path exists
    if not os.path.exists(path):
        raise ValueError(f"The provided path '{path}' does not exist.")
    # folder_tree = f"{start_folder_name} (h1)\n"

    path = os.path.abspath(path)

    # for root, dir, files in os.walk(path):
    #     for file in files:
    #         if file.endswith(".pkl"):
    #             path_list = [start_folder_name] + string_subtraction(root, path).split(
    #                 "/"
    #             )[1:]
    #             line = (
    #                     (len(path_list) - 1) * "--"
    #                     + path_list[-1]
    #                     + f" (L{len(path_list)})"
    #             )
    #             folder_tree += f"{line}\n"

    for root, dir, files in os.walk(path):
        for file in files:
            if file.endswith(".pkl"):
                # file path
                file_path = os.path.join(root, file)
                path_list = [start_folder_name] + string_subtraction(root, path).split(
                    "/"
                )[1:]

                # Find associated file (pdf, video, or md) in the same directory
                base_name = os.path.splitext(file)[0]
                md_path = os.path.join(root, f"{base_name}.md")

                # Determine which file path to use based on priority
                File_path = None
                if os.path.exists(md_path):
                    File_path = md_path

                # Convert absolute path to relative path including the root folder name
                if File_path:
                    # Get path relative to the parent directory of the root folder
                    parent_dir = os.path.dirname(path)
                    relative_file_path = os.path.relpath(File_path, parent_dir)
                else:
                    relative_file_path = None

                with open(file_path, "rb") as pkl_file:
                    print(file_path)
                    chunks = pickle.load(pkl_file)
                for chunk in chunks:
                    topic_path = " > ".join(chunk.titles)
                    topic_path_list.append(topic_path)
                    print(f"topic_path: {topic_path} in {file_path}")
                    id = base_name + ' > ' + topic_path
                    id_list.append(id)
                    doc_list.append(chunk.content)
                    url = "".join(chunk.chunk_url)
                    url_list.append(url)
                    file_paths_list.append(relative_file_path)
                    index_list.append(chunk.index)
    return url_list, id_list, doc_list, file_paths_list, topic_path_list, index_list


def embedding_create(markdown_path, name, embedding_name, folder_name, model):
    """
    Traverse through files
    """
    fail = []
    start = time.time()
    # Process each page
    url_list, id_list, doc_list, file_paths_list, topic_path_list, index_list = traverse_files(
        markdown_path,
        name
    )
    if model == "BGE":
        embedding_model = BGEM3FlagModel("BAAI/bge-m3", use_fp16=True)
    human_embedding_prompt = (
        "document_hierarchy_path: {segment_path}\ndocument: {segment}\n"
    )
    hp_list = [human_embedding_prompt.format(segment=doc, segment_path=idid) for doc, idid in zip(doc_list, id_list)]
    embedding_list = embedding_model.encode(
        hp_list, return_dense=True, return_sparse=True, return_colbert_vecs=True
    )

    id_list = np.array(id_list)
    doc_list = np.array(doc_list)
    url_list = np.array(url_list)
    file_paths_list = np.array(file_paths_list)
    topic_path_list = np.array(topic_path_list)
    index_list=np.array(index_list)
    print("create time:", time.time() - start)
    # Store the variables in a dictionary
    data_to_store = {
        "id_list": id_list,
        "doc_list": doc_list,
        "embedding_list": embedding_list,
        "url_list": url_list,
        "file_paths_list": file_paths_list,
        "topic_path_list": topic_path_list,
        "index_list": index_list
    }

    validate_data(data_to_store)

    # Create the folder if it does not exist
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    with open(f"{folder_name}/{embedding_name}.pkl", "wb") as f:
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
    # Assume 'data' is your dictionary and 'content_logger' is your logger
    expected_length = None
    lengths = []

    for key, value in data.items():
        # Check if value is empty
        if len(value) == 0:
            content_logger.error(f"{key} is empty.")
            continue  # Skip further checks if empty

        # Check if the value is a numpy array
        if not isinstance(value, np.ndarray):
            content_logger.error(f"{key} is not a numpy array.")

        # Establish the expected length from the first non-empty value
        if expected_length is None:
            expected_length = len(value)

        # Check if the current value's length matches the expected length
        if len(value) != expected_length:
            content_logger.error(
                f"{key} length {len(value)} is not equal to expected length {expected_length}."
            )

        lengths.append(len(value))

    if len(set(lengths)) > 1:
        content_logger.error("Lists are of inconsistent lengths.")

    content_logger.info("Data validation complete.")


if __name__ == "__main__":
    embedding_create(
        "/home/bot/bot/yk/YK_final/ROAR-Academy-main_output",
        "/home/bot/bot/yk/YK_final/ROAR-Academy-main_output",
        "ROAR-Academy",
        "embedding_output",
        "BGE",
    )
