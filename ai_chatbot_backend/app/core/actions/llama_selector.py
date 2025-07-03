import os
import pickle
import numpy as np
from dotenv import load_dotenv
from FlagEmbedding import BGEM3FlagModel
import transformers
import torch
from threading import Thread
from typing import List, Union, Any
from app.core.models.chat_completion import Message as ROARChatCompletionMessage
from pydantic import BaseModel
import threading
import urllib.parse
import json
import sqlite3

# Configuration Flags
SQLDB = False  # Set to True if using SQLDB for embeddings
EXT_VECTOR_PATH = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vector0"
EXT_VSS_PATH = "ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug/vss0"


class Message(BaseModel):
    role: str
    content: str


# Load Environment Variables
load_dotenv()

# Initialize Embedding Model
embedding_model = BGEM3FlagModel("BAAI/bge-m3", use_fp16=True)

# Initialize Transformer Model and Tokenizer
model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
auto_tokenizer = transformers.AutoTokenizer.from_pretrained(model_id)
print("Loading model...")
pipeline = transformers.pipeline(
    "text-generation",
    model=model_id,
    model_kwargs={"torch_dtype": torch.bfloat16},
    device="cuda",
)

# Thread Lock for Model Access
lock = threading.Lock()


def prompt_generator(
    messages: List[Message], streamer_iterator: transformers.TextIteratorStreamer
):
    """
    Generates prompts for the model in a separate thread.

    Args:
        messages (List[Message]): List of conversation messages.
        streamer_iterator (transformers.TextIteratorStreamer): Streamer for handling output.
    """
    with lock:
        terminators = [
            pipeline.tokenizer.eos_token_id,
            pipeline.tokenizer.convert_tokens_to_ids("<|eot_id|>"),
        ]
        prompt = pipeline.tokenizer.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )

        pipeline(
            prompt,
            max_new_tokens=1000,
            eos_token_id=terminators,
            do_sample=True,
            streamer=streamer_iterator,
        )


def bge_compute_score(
    query_embedding: dict,
    document_embeddings: List[dict],
    weights_for_different_modes: List[float],
    secondary_query_embedding: Any = None,
    secondary_document_embeddings: Any = None,
) -> dict:
    """
    Computes scores for RAG based on different embedding modes.

    Args:
        query_embedding (dict): Embeddings of the query.
        document_embeddings (List[dict]): List of document embeddings.
        weights_for_different_modes (List[float]): Weights for dense, sparse, and colbert modes.
        secondary_query_embedding (Any, optional): Secondary query embeddings.
        secondary_document_embeddings (Any, optional): Secondary document embeddings.

    Returns:
        dict: Computed scores for each mode.
    """
    all_scores = {
        "colbert": [],
        "sparse": [],
        "dense": [],
        "sparse+dense": [],
        "colbert+sparse+dense": [],
    }

    if weights_for_different_modes is None:
        weights_for_different_modes = [1, 1.0, 1.0]
        weight_sum = 3
        print("Default weights for dense, sparse, colbert are [1.0, 1.0, 1.0]")
    else:
        assert len(weights_for_different_modes) == 3, (
            "Weights list must have exactly 3 elements."
        )
        weight_sum = sum(weights_for_different_modes)

    for i in range(len(document_embeddings)):
        dense_score = np.dot(
            query_embedding["dense_vecs"], document_embeddings[i]["dense_vecs"]
        )
        sparse_score = embedding_model.compute_lexical_matching_score(
            query_embedding["lexical_weights"],
            document_embeddings[i]["lexical_weights"],
        )
        colbert_score = embedding_model.colbert_score(
            query_embedding["colbert_vecs"], document_embeddings[i]["colbert_vecs"]
        )

        all_scores["colbert"].append(colbert_score)
        all_scores["sparse"].append(sparse_score)
        all_scores["dense"].append(dense_score)
        all_scores["sparse+dense"].append(
            (
                sparse_score * weights_for_different_modes[1]
                + dense_score * weights_for_different_modes[0]
            )
            / (weights_for_different_modes[1] + weights_for_different_modes[0])
        )
        all_scores["colbert+sparse+dense"].append(
            (
                colbert_score * weights_for_different_modes[2]
                + sparse_score * weights_for_different_modes[1]
                + dense_score * weights_for_different_modes[0]
            )
            / weight_sum
        )

    return all_scores


def clean_path(url_path: str) -> str:
    """
    Cleans and formats the URL path.

    Args:
        url_path (str): The URL path to clean.

    Returns:
        str: Cleaned URL path.
    """
    decoded_path = urllib.parse.unquote(url_path)
    cleaned_path = (
        decoded_path.replace("%28", "(").replace("%29", ")").replace("%2B", "+")
    )
    cleaned_path = cleaned_path.replace(">", " > ")
    cleaned_path = cleaned_path.replace("(", " (").replace(")", ") ")
    cleaned_path = " ".join(cleaned_path.split())
    return cleaned_path


def process_references(
    top_docs: List[str], top_ids: List[str], top_urls: List[str], distances: List[float]
) -> (str, List[str]):
    """
    Processes top documents and references to generate insertion text.

    Args:
        top_docs (List[str]): List of top documents.
        top_ids (List[str]): List of top document IDs.
        top_urls (List[str]): List of top document URLs.
        distances (List[float]): List of distances/scores.

    Returns:
        tuple: Inserted document string and list of references.
    """
    insert_document = ""
    reference = []
    n = 0
    reference_string = ""
    for i in range(len(top_docs)):
        if top_urls[i]:
            reference.append(f"{top_urls[i]}")
        else:
            reference.append("")
        if distances[i] > 0.38:
            n += 1
            if top_urls[i]:
                insert_document += f'"""Reference Number: {n}\nReference Info Path(not URL): {top_ids[i]}\nDocument: {top_docs[i]}"""\n\n'
                cleaned_path = clean_path(top_ids[i])
                reference_string += f"Reference {n}: <|begin_of_reference_name|>{cleaned_path}<|end_of_reference_name|><|begin_of_reference_link|>{top_urls[i]}<|end_of_reference_link|>\n\n"
            else:
                cleaned_path = clean_path(top_ids[i])
                insert_document += f'"""Reference Number: {n}\nReference Info Path(not URL): {cleaned_path}\nDocument: {top_docs[i]}"""\n\n'
                reference_string += f"Reference {n}: <|begin_of_reference_name|>{cleaned_path}<|end_of_reference_name|><|begin_of_reference_link|><|end_of_reference_link|>\n\n"
        else:
            reference.append("")

    return insert_document, reference


def perform_rag(messages: List[Message], course: str) -> Any:
    """
    Perform Retrieval-Augmented Generation (RAG) based on the provided messages and course.

    Args:
        messages (List[Message]): List of messages in the conversation.
        course (str): The course identifier for selecting relevant documents.

    Returns:
        Any: The response from the RAG process.
    """
    insert_document = ""
    user_message = messages[-1].content

    # Map courses to their respective pickle files
    course_pickle_map = {
        "EE 106B": ["eecs106b.pkl", "Robotic Manipulation and Interaction"],
        "CS 61A": ["cs61a.pkl", "Structure and Interpretation of Computer Programs"],
        "CS 294-137": [
            "cs294.pkl",
            "Immersive Computing and Virtual Reality using Unity",
        ],
        "Econ 140": ["Econ140.pkl", "Econometrics"],
        "ROAR Academy": ["roar_academy.pkl", "ROAR Academy"],
        "Multilingual Engagement": ["language.pkl", "Multilingual_Engagement"],
        "Default": ["Berkeley.pkl", "Berkeley"],
    }
    picklefile, class_name = course_pickle_map.get(course, ["Berkeley.pkl", "Berkeley"])
    current_dir = (
        "/home/bot/localgpt/tai/ai_chatbot_backend/app/embedding/"  # Modify as needed
    )
    num_docs = 7
    if SQLDB:
        # SQLDB logic
        embedding_db_path = os.path.join(current_dir, "embeddings.db")

        # Connect to the embeddings database using vss and vector extensions
        db = sqlite3.connect(embedding_db_path)
        db.enable_load_extension(True)
        db.load_extension(EXT_VECTOR_PATH)
        db.load_extension(EXT_VSS_PATH)
        cur = db.cursor()

        # Encode the user message
        query_embed = embedding_model.encode(
            user_message,
            return_dense=True,
            return_sparse=True,
            return_colbert_vecs=True,
        )
        query_vector = query_embed["dense_vecs"].tolist()
        query_vector_json = json.dumps(query_vector)

        # Query the embeddings database using vss_search
        cur.execute(
            """
            SELECT 
                rowid, 
                distance
            FROM embeddings
            WHERE vss_search(
                embedding,
                ?
            )
            LIMIT 3;
        """,
            (query_vector_json,),
        )

        results = cur.fetchall()
        db.commit()
        db.close()

        # Connect to the main database to extract the top docs and urls
        table_name = picklefile.replace(".pkl", "")
        db_name = f"{table_name}.db"
        main_db_path = os.path.join(current_dir, db_name)
        db = sqlite3.connect(main_db_path)
        cur = db.cursor()

        # Extract the top 3 docs and urls
        indices = [result[0] for result in results]
        distances = [result[1] for result in results]

        placeholders = ",".join("?" for _ in indices)
        query = f"SELECT id_list, doc_list, url_list FROM {table_name} WHERE rowid IN ({placeholders})"
        cur.execute(query, indices)
        results = cur.fetchall()

        top_ids, top_docs, top_urls = [], [], []
        for id, doc, url in results:
            top_ids.append(id)
            top_docs.append(doc)
            top_urls.append(url)

        db.close()

    else:
        # Picklefile implementation
        path_to_pickle = os.path.join(current_dir, picklefile)
        with open(path_to_pickle, "rb") as f:
            data_loaded = pickle.load(f)

        doc_list = data_loaded["doc_list"]
        id_list = data_loaded["id_list"]
        url_list = data_loaded["url_list"]
        embedding_list = data_loaded["embedding_list"]

        # Encode the user message
        query_embed = embedding_model.encode(
            user_message,
            return_dense=True,
            return_sparse=True,
            return_colbert_vecs=True,
        )

        # Compute cosine similarities
        scores = bge_compute_score(
            query_embedding=query_embed,
            document_embeddings=embedding_list,
            weights_for_different_modes=[1, 1, 1],
        )
        cosine_similarities = np.array(scores["colbert+sparse+dense"])
        indices = np.argsort(cosine_similarities)[::-1]
        distances = cosine_similarities[indices][:num_docs]

        top_ids = [id_list[i] for i in indices[:num_docs]]
        top_docs = [doc_list[i] for i in indices[:num_docs]]
        top_urls = [url_list[i] for i in indices[:num_docs]]

    print("Top IDs:", top_ids)
    print("Top Docs:", top_docs)
    print("Top URLs:", top_urls)

    # Process top documents and references
    insert_document, reference = process_references(
        top_docs, top_ids, top_urls, distances
    )

    if not insert_document:
        print("NO REFERENCES")
        user_message = f"Answer the instruction. If unsure of the answer, explain that there is no data in the knowledge base for the response and refuse to answer. If the instruction is not related to class topic {class_name}, explain and refuse to answer.\n---\nInstruction: {user_message}"
    else:
        print("INSERT DOCUMENT:", insert_document)
        insert_document += f"Instruction: {user_message}"
        user_message = f"Understand the reference documents and use related ones to answer the instruction thoroughly. Keep your answer ground in the facts of the references that are relevant and refer to spacific reference number inline. Do not provide any reference at the end. If the instruction is not related to class topic {class_name}, explain and refuse to answer.\n---\n{insert_document}"

    print("USER MESSAGE:", user_message)
    messages[-1].content = user_message

    streamer_iterator = transformers.TextIteratorStreamer(
        auto_tokenizer, skip_prompt=True
    )
    t = Thread(
        target=prompt_generator,
        args=(
            messages,
            streamer_iterator,
        ),
    )
    t.start()
    response = streamer_iterator
    return response, reference


def perform_direct_query(
    messages: List[Message], stream: bool = False
) -> Union[str, transformers.TextIteratorStreamer]:
    """
    Directly query the model without using RAG.

    Args:
        messages (List[Message]): List of messages in the conversation.
        stream (bool): Whether to stream the response.

    Returns:
        Union[str, transformers.TextIteratorStreamer]: The model's response.
    """
    formatted_messages = local_formatter(messages)

    if stream:
        streamer_iterator = transformers.TextIteratorStreamer(
            auto_tokenizer, skip_prompt=True
        )
        t = Thread(
            target=prompt_generator,
            args=(
                formatted_messages,
                streamer_iterator,
            ),
        )
        t.start()
        return streamer_iterator
    else:
        response = pipeline(formatted_messages, max_new_tokens=1000, do_sample=True)
        return response[0]["generated_text"]


def local_selector(
    messages: List[Message], stream: bool = True, rag: bool = True, course: str = None
) -> Union[str, transformers.TextIteratorStreamer]:
    """
    Selects the appropriate model querying method based on the RAG flag.

    Args:
        messages (List[Message]): List of messages in the conversation.
        stream (bool): Whether to stream the response.
        rag (bool): Whether to use RAG.
        course (str): The course identifier for selecting relevant documents.

    Returns:
        Union[str, transformers.TextIteratorStreamer]: The model's response.
    """
    if rag:
        return perform_rag(messages, course)
    return perform_direct_query(messages, stream), None


def local_parser(stream: transformers.TextIteratorStreamer, reference_string: str):
    """
    Parses the streamed response by removing specific tokens.

    Args:
        stream (transformers.TextIteratorStreamer): Streamed response from the model.

    Yields:
        str: Parsed text chunks.
    """
    for chunk in stream:
        result = chunk.replace("<|eot_id|>", "")
        if result == None:
            yield ""
        else:
            yield chunk.replace("<|eot_id|>", "")
    yield "\n\n<|begin_of_reference|>\n\n" + reference_string + "<|end_of_reference|>"


def local_formatter(messages: List[ROARChatCompletionMessage]) -> List[Message]:
    """
    Formats the incoming messages by adding a system prompt.

    Args:
        messages (List[ROARChatCompletionMessage]): Incoming conversation messages.

    Returns:
        List[Message]: Formatted messages with system prompt.
    """
    response: List[Message] = []
    system_message = "You are a Teaching Assistant. You are responsible for answering questions and providing guidance to students. Do not provide direct answers to homework questions. If the question is related to a class topic, provide guidance and resources to help the student answer the question. If the question is not related to a class topic, explain that you cannot provide an answer."
    response.append(Message(role="system", content=system_message))
    for message in messages:
        response.append(Message(role=message.role, content=message.content))
    return response
