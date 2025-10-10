# Standard python libraries
import time
from typing import Any, Optional, Tuple, List, Dict
from uuid import UUID
# Local libraries
from app.services.rag_retriever import get_reference_documents, get_chunks_by_file_uuid, get_sections_by_file_uuid, get_file_related_documents
from app.services.rag_postprocess import extract_channels

async def build_retrieval_query(user_message: str, memory_synopsis: Any, engine: Any, tokenizer: Any, sampling: Any, file_sections: Any = None, excerpt: Any = None) -> str:
    """
    Reformulate the latest user request into a single self-contained query string,
    based on the full chat history (user + assistant messages).
    Returns plain text with no quotes or extra formatting.
    """
    # Prepare the chat history for the model
    system_prompt = (
        "You are a query reformulator for a RAG system. "
        "Given the user message and the memory synopsis of the current conversation as well as the file context if any, "
        "rewrite the latest user request as a single, "
        "self-contained question for document retrieval. "
        "Resolve pronouns and references using context, include relevant constraints "
        "(dates, versions, scope), and avoid adding facts not in the history. "
        "Return only the rewritten query as question in plain text—no quotes, no extra text."
        "# Valid channels: analysis, commentary, final. Channel must be included for every message."
        "Calls to these tools must go to the commentary channel: 'functions'.<|end|>"
    )

    # If no context is provided, return the original user message
    if not memory_synopsis and not file_sections and not excerpt:
        return user_message

    request_parts = []

    if memory_synopsis:
        request_parts.append(f"Memory Synopsis:\n{memory_synopsis}\n")
    
    if file_sections or excerpt:
        request_parts.append(f"File Context:\n")

    if file_sections:
        request_parts.append(f"The user is looking at this file which has these sections: {file_sections}\n")

    if excerpt:
        request_parts.append(f"The user is focused on the following part of the file: {excerpt}\n")

    request_parts.append(f"User Message:\n{user_message}\n")

    request_content = "\n".join(request_parts)

    chat = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": request_content}
    ]
    prompt = tokenizer.apply_chat_template(chat, tokenize=False, add_generation_prompt=True)
    # Generate the query using the engine
    text = ""
    async for chunk in engine.generate(
            prompt=prompt,
            sampling_params=sampling,
            request_id=str(time.time_ns())
    ):
        text = chunk.outputs[0].text
    text = extract_channels(text).get('final', "{}")
    print(f"[INFO] Generated RAG-Query: {text.strip()}")
    return text


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
        audio_response: bool = False
) -> Tuple[str, List[Dict], str]:
    """
    Build an augmented prompt by retrieving reference documents.
    Returns:
      - modified_message: the augmented instruction prompt.
      - reference_list: list of reference URLs for JSON output.
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
    ), class_name = get_reference_documents(query_message, course, top_k=top_k)
    # Prepare the insert document and reference list
    insert_document = ""
    reference_list = reference_list or []
    n = len(reference_list)
    for i in range(len(top_docs)):
        if similarity_scores[i] > threshold:
            n += 1
            file_path = top_files[i]
            file_uuid = top_file_uuids[i]
            chunk_index = top_chunk_idxs[i]
            topic_path = top_refs[i]
            url = top_urls[i] if top_urls[i] else ""
            insert_document += (
                f'Reference Number: {n}\n'
                f"Directory Path to reference file to tell what file is about: {file_path}\n"
                f"Topic Path of chunk in file to tell the topic of chunk: {topic_path}\n"
                f'Document: {top_docs[i]}\n\n'
            )
            reference_list.append([topic_path, url, file_path, file_uuid,chunk_index])
    # Create response and reference styles based on audio mode.
    if not audio_response:
        response_style = (
            f"Answer the instruction thoroughly with a well-structured markdown format answer. "
            f"No references at the end."
        )
        reference_style = (
            f"\nALWAYS: Refer to specific reference numbers inline using [Reference: n] style!!! Do not use other style like refs, 【】, Reference: [n], > *Reference: n*  or (reference n)!!!"
            f"\nDo not list references at the end. "
        )
    else:
        response_style = """
        STYLE:
        Use a clear, natural, speaker-friendly tone that is short and engaging. Try to end every sentence with a period '.'. ALWAYS: Avoid code block, Markdown formatting or math equation!!! No references at the end or listed withou telling usage.
        Make the first sentence short and engaging. If no instruction is given, explain that you did not hear any instruction.
        Do not use symbols that are not readable in speech, such as (, ), [, ], {, }, <, >, *, #, -, !, $, %, ^, &, =, +, \, /, ~, `, etc. In this way, avoid code, Markdown formatting or math equation!!!
        """
        reference_style = (
            "\nREFERENCE USAGE:"
            f"\nMention specific reference numbers inline when that part of the answer is refer to some reference. "
            f"\nALWAYS: Do not mention references in a unreadable format like refs, 【】, Reference: [n], > *Reference: n* or (reference n)!!! Those are not understandable since the output is going to be converted to speech. "
            f"\nGood example: According to reference 1, as mention in reference 2, etc. \n"
        )
    # Create modified message based on whether documents were inserted
    if not insert_document or n == 0:
        system_add_message = (
            f"\n{response_style}"
            f"If the question is a complex question, provide hints, explanations, "
            f"or step-by-step guidance instead of a direct answer. "
            f"If you are unsure after making a reasonable effort, "
            f"explain that there is no data in the knowledge base for the response. "
            f"Only refuse if the question is clearly unrelated to any topic in "
            f"{course}: {class_name} and is not a general, reasonable query. "
            f"If the intent is unclear, ask clarifying questions rather than refusing."
        )
        modified_message = (
            f""
        )
    else:
        system_add_message =(
            f"\n{response_style}"
            f"Review the reference documents, considering their Directory Path (original file location), "
            f"Topic Path (section or title it belongs to), and Document content. "
            f"Select only the most relevant references to answer the instruction thoroughly "
            f"in a well-structured markdown format (do not add '```markdown'). "
            f"Ground your answer in the facts from these selected references. "
            f"If the question is a complex problem, provide hints, explanations, "
            f"or step-by-step guidance instead of giving the direct answer. "
            f"{reference_style}"
            f"Exclude and avoid explaining irrelevant references. "
            f"If, after reasonable effort, no relevant information is found, "
            f"state that there is no data in the knowledge base for the response. "
            f"Refuse only if the question is clearly unrelated to any topic in {course}: {class_name}, "
            f"is not a general query, and has no link to the provided references. "
            f"If intent is unclear, ask clarifying questions before refusing."
        )
        modified_message = (
            f"{insert_document}\n---\n"
        )
    # Append user instruction to the modified message
    if not (answer_content and problem_content):
        modified_message += f"Instruction: {user_message}"
    else:
        modified_message += user_message
    # Return the final modified message and reference list
    return modified_message, reference_list, system_add_message


def build_file_augmented_context(
    file_uuid: UUID,
    selected_text: Optional[str] = None,
    index: Optional[float] = None,
) -> Tuple[str, str, str, List[Dict]]:
    """
    Build an augmented context for file-based chat by retrieving reference documents.
    Returns:
    - augmented_context: the augmented context for the file.
    - focused_chunk: the chunk of text the user is currently focused on.
    - file_content: the full content of the file.
    - sections: list of sections in the file.
    """
    # Get file content by file UUID
    chunks = get_chunks_by_file_uuid(file_uuid)
    sections = get_sections_by_file_uuid(file_uuid)
    file_content = " ".join(chunk["chunk"] for chunk in chunks)

    augmented_context = (
        f"The user is looking at this file to give the instruction: \n{file_content}\n---\n"
    )

    focused_chunk = ""
    if index:
        # Find chunks closest to the given index
        closest_chunks = []
        if chunks:
            # Calculate distances and find minimum distance
            min_distance = min(abs(chunk['index'] - index) for chunk in chunks)
            # Get all chunks with minimum distance
            closest_chunks = [chunk for chunk in chunks if abs(chunk['index'] - index) == min_distance]
            # Already sorted by chunk_index, so closest_chunks are in order
        focused_chunk = ' '.join(chunk['chunk'] for chunk in closest_chunks)
        augmented_context += f"The user is focused on the following part of the file: {focused_chunk}\n\n"
    
    if selected_text:
        augmented_context += f"The user has selected the following text in the file:\n\n{selected_text}\n\n"

    return augmented_context, file_content, focused_chunk, sections

    # The following code is commented out because file chat no longer needs separate reference retrieval for file-related context.

    # if not rag:
    #     return augmented_context, []

    # # Get reference documents based on the selected text.
    # (
    #     top_chunk_uuids, top_docs, top_urls, similarity_scores, top_files, top_refs, top_titles,
    #     top_file_uuids, top_chunk_idxs
    # ), _ = get_reference_documents(selected_text, course, top_k=top_k // 2) if selected_text else ([], [], [], [], [], [], [],[], []), ""

    # # Get reference documents based on the entire document.
    # (
    #     top_chunk_uuids_doc, top_docs_doc, top_urls_doc, similarity_scores_doc, top_files_doc, top_refs_doc, top_titles_doc,
    #     top_file_uuids_doc, top_chunk_idxs_doc
    # ) = get_file_related_documents(file_uuid, course, top_k=top_k // 2 if selected_text else top_k)

    # # Combine results from selected text and entire document
    # top_ids_combined = top_chunk_uuids + top_chunk_uuids_doc
    # top_docs_combined = top_docs + top_docs_doc
    # top_urls_combined = top_urls + top_urls_doc
    # similarity_scores_combined = similarity_scores + similarity_scores_doc
    # top_files_combined = top_files + top_files_doc
    # top_refs_combined = top_refs + top_refs_doc
    # top_titles_combined = top_titles + top_titles_doc
    # top_file_uuids_combined = top_file_uuids + top_file_uuids_doc
    # top_chunk_idxs_combined = top_chunk_idxs + top_chunk_idxs_doc

    # insert_document = ""
    # reference_list = reference_list or []
    # n = len(reference_list)
    # for i in range(len(top_docs_combined)):
    #     if similarity_scores_combined[i] > threshold:
    #         n += 1
    #         file_path = top_files_combined[i]
    #         file_uuid = top_file_uuids_combined[i]
    #         chunk_index = top_chunk_idxs_combined[i]
    #         topic_path = top_refs_combined[i]
    #         url = top_urls_combined[i] if top_urls_combined[i] else ""
    #         insert_document += (
    #             f'Reference Number: {n}\n'
    #             f"Directory Path to reference file to tell what file is about: {file_path}\n"
    #             f"Topic Path of chunk in file to tell the topic of chunk: {topic_path}\n"
    #             f'Document: {top_docs_combined[i]}\n\n'
    #         )
    #         reference_list.append([topic_path, url, file_path, file_uuid, chunk_index])

    # augmented_context += insert_document + "---\n"
    # return augmented_context, reference_list