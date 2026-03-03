import time
from typing import Dict, List, Optional, Tuple

from app.services.query.vector_search import get_reference_documents, get_file_descriptions_by_uuids
from app.services.generation.prompts import modes
from app.services.request_timer import RequestTimer


def build_augmented_prompt(
        user_message: str,
        course: str,
        threshold: float,
        rag: bool,
        top_k: int = 7,
        query_message: str = "",
        reference_list: List[Dict] = None,
        problem_content: Optional[str] = None,
        answer_content: Optional[str] = None,
        audio_response: bool = False,
        tutor_mode: bool = True,
        timer: Optional[RequestTimer] = None
) -> Tuple[str, List[Dict], str]:
    """
    Build an augmented prompt by retrieving reference documents.

    4-Mode System:
    - Chat Tutor (tutor_mode=True, audio_response=False): JSON with citations-first
    - Chat Regular (tutor_mode=False, audio_response=False): Plain Markdown
    - Voice Tutor (tutor_mode=True, audio_response=True): JSON with unreadable property
    - Voice Regular (tutor_mode=False, audio_response=True): Plain speakable text

    Returns:
      - modified_message: the augmented instruction prompt.
      - reference_list: list of reference URLs for JSON output.
      - system_add_message: additional system message content.
    """
    # Practice mode has its own message format
    if answer_content and problem_content:
        user_message = (
            f"Course problem:\n{problem_content}\n"
            f"Answer attempted by user:\n{answer_content}\n"
            f"Instruction: {user_message}"
        )
    # Print parameter information
    print('\n Course: \n', course, '\n')
    print("\nUser Question: \n", user_message, "\n")
    print('time of the day:', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()), '\n')
    # No need to retrieve documents if rag is False
    if not rag:
        return user_message, []
    # If query_message is not provided, use user_message
    if not query_message:
        query_message = user_message
    # Retrieve reference documents based on the query
    (
        top_chunk_uuids, top_docs, top_urls, similarity_scores, top_files, top_refs, top_titles,
        top_file_uuids, top_chunk_idxs
    ), class_name = get_reference_documents(query_message, course, top_k=top_k, timer=timer)
    # Prepare the insert document and reference list
    # Collect file UUIDs that pass threshold, then batch-fetch their descriptions
    passing_indices = [i for i in range(len(top_docs)) if similarity_scores[i] > threshold]
    file_desc_map = {}
    if passing_indices:
        uuids_needed = [top_file_uuids[i] for i in passing_indices]
        file_desc_map = get_file_descriptions_by_uuids(uuids_needed)

    insert_document = ""
    reference_list = reference_list or []
    n = len(reference_list)
    for i in passing_indices:
        n += 1
        file_path = top_files[i]
        file_uuid = top_file_uuids[i]
        chunk_index = top_chunk_idxs[i]
        topic_path = top_refs[i]
        url = top_urls[i] if top_urls[i] else ""
        file_desc = file_desc_map.get(file_uuid, "")
        insert_document += (
            f'Reference Number: {n}\n'
            f"Directory Path to reference file to tell what file is about: {file_path}\n"
        )
        if file_desc:
            insert_document += f"File Description: {file_desc}\n"
        insert_document += (
            f"Topic Path of chunk in file to tell the topic of chunk: {topic_path}\n"
            f'Document: {top_docs[i]}\n\n'
        )
        reference_list.append([topic_path, url, file_path, file_uuid, chunk_index])

    # Get mode configuration - single source of truth
    config = modes.get_mode_config(tutor_mode, audio_response)

    # Select complete system prompt based on whether documents were found
    if not insert_document or n == 0:
        print("[INFO] No relevant documents found above the similarity threshold.")
        system_prompt = config.system_prompt_no_refs
        modified_message = ""
    else:
        print("[INFO] Relevant documents found and inserted into the prompt.")
        system_prompt = config.system_prompt_with_refs
        modified_message = f"{insert_document}\n---\n"
    # Resolve {course}/{class_name} placeholders
    system_add_message = system_prompt.format(course=course, class_name=class_name)
    # Append user instruction to the modified message
    if not (answer_content and problem_content):
        modified_message += f"Instruction: {user_message}"
    else:
        modified_message += user_message
    # Return the final modified message and reference list
    return modified_message, reference_list, system_add_message


def build_prompt_from_refs(
    user_message: str,
    course: str,
    class_name: str,
    refs: tuple,
    problem_content: Optional[str] = None,
    answer_content: Optional[str] = None,
    audio_response: bool = False,
    tutor_mode: bool = True,
    reference_list: List[Dict] = None,
    outline_mode: bool = False,
) -> Tuple[str, List[Dict], str]:
    """
    Format pre-fetched references into an augmented prompt.

    Unlike build_augmented_prompt, this function does NOT perform retrieval —
    it receives already-filtered references (9-tuple) from the caller.

    Args:
        refs: 9-tuple (chunk_uuids, texts, urls, scores, file_paths,
              reference_paths, titles, file_uuids, chunk_idxs)
              — all entries already passed the threshold.
        outline_mode: if True, use outline system prompt instead of block-based.
    """
    # Practice mode formatting
    if answer_content and problem_content:
        user_message = (
            f"Course problem:\n{problem_content}\n"
            f"Answer attempted by user:\n{answer_content}\n"
            f"Instruction: {user_message}"
        )

    (top_chunk_uuids, top_docs, top_urls, similarity_scores,
     top_files, top_refs, top_titles, top_file_uuids, top_chunk_idxs) = refs

    # Batch-fetch file descriptions
    file_desc_map = {}
    if top_file_uuids:
        file_desc_map = get_file_descriptions_by_uuids(list(set(top_file_uuids)))

    insert_document = ""
    reference_list = reference_list or []
    n = len(reference_list)
    for i in range(len(top_docs)):
        n += 1
        file_path = top_files[i]
        file_uuid = top_file_uuids[i]
        chunk_index = top_chunk_idxs[i]
        topic_path = top_refs[i]
        url = top_urls[i] if top_urls[i] else ""
        file_desc = file_desc_map.get(file_uuid, "")
        insert_document += (
            f'Reference Number: {n}\n'
            f"Directory Path to reference file to tell what file is about: {file_path}\n"
        )
        if file_desc:
            insert_document += f"File Description: {file_desc}\n"
        insert_document += (
            f"Topic Path of chunk in file to tell the topic of chunk: {topic_path}\n"
            f'Document: {top_docs[i]}\n\n'
        )
        reference_list.append([topic_path, url, file_path, file_uuid, chunk_index])

    # Get mode configuration
    if outline_mode:
        config = modes.get_outline_mode_config()
    else:
        config = modes.get_mode_config(tutor_mode, audio_response)

    # Select system prompt based on whether documents were found
    if not insert_document or n == 0:
        print("[INFO] No relevant documents found above the similarity threshold.")
        system_prompt = config.system_prompt_no_refs
        modified_message = ""
    else:
        print("[INFO] Relevant documents found and inserted into the prompt.")
        system_prompt = config.system_prompt_with_refs
        modified_message = f"{insert_document}\n---\n"

    system_add_message = system_prompt.format(course=course, class_name=class_name)

    if not (answer_content and problem_content):
        modified_message += f"Instruction: {user_message}"
    else:
        modified_message += user_message

    return modified_message, reference_list, system_add_message
