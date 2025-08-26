# Standard python libraries
import time
from typing import Any, Optional, Tuple, List
# Local libraries
from app.services.rag_retriever import get_reference_documents


async def build_retrieval_query(user_message: str, memory_synopsis: Any, engine: Any, tokenizer: Any, sampling: Any) -> str:
    """
    Reformulate the latest user request into a single self-contained query string,
    based on the full chat history (user + assistant messages).
    Returns plain text with no quotes or extra formatting.
    """
    # Prepare the chat history for the model
    system_prompt = (
        "You are a query reformulator for a RAG system. "
        "Given the user message and the memory synopsis of the current conversation, "
        "rewrite the latest user request as a single, "
        "self-contained question for document retrieval. "
        "Resolve pronouns and references using context, include relevant constraints "
        "(dates, versions, scope), and avoid adding facts not in the history. "
        "Return only the rewritten query as question in plain text—no quotes, no extra text."
    )

    request_template = """Memory Synopsis:
    {memory_synopsis}

    User Message:
    {user_message}
    """

    chat = [
        {"role": "system", "content": system_prompt},
        {
            "role": "user",
            "content": request_template.format(
                memory_synopsis=memory_synopsis.to_json() if memory_synopsis else "",
                user_message=user_message
            )
        }
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
    print(f"[INFO] Generated RAG-Query: {text.strip()}")
    return text


def build_augmented_prompt(
        user_message: str,
        course: str,
        threshold: float,
        rag: bool,
        top_k: int = 7,
        query_message: str = "",
        practice: bool = False,
        problem_content: Optional[str] = None,
        answer_content: Optional[str] = None,
        audio_response: bool = False
) -> Tuple[str, List[str | Any]]:
    """
    Build an augmented prompt by retrieving reference documents.
    Returns:
      - modified_message: the augmented instruction prompt.
      - reference_list: list of reference URLs for JSON output.
    """
    # Practice mode has its own message format
    if practice:
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
        top_ids,
        top_docs,
        top_urls,
        similarity_scores,
        top_files,
        top_refs,
        top_titles
    ), class_name = get_reference_documents(query_message, course, top_k=top_k)
    # Prepare the insert document and reference list
    insert_document = ""
    reference_list: List[str | Any] = []
    n = 0
    for i in range(len(top_docs)):
        if similarity_scores[i] > threshold:
            n += 1
            file_path = top_files[i]
            topic_path = top_refs[i]
            url = top_urls[i] if top_urls[i] else ""
            insert_document += (
                f'Reference Number: {n}\n'
                f"Directory Path to reference file to tell what file is about: {file_path}\n"
                f"Topic Path of chunk in file to tell the topic of chunk: {topic_path}\n"
                f'Document: {top_docs[i]}\n\n'
            )
            reference_list.append([topic_path, url, file_path])
    # Create response and reference styles based on audio mode.
    if not audio_response:
        response_style = (
            f"Answer the instruction thoroughly with a well-structured markdown format answer. "
            f"No references at the end."
        )
        reference_style = (
            f"Refer to specific reference numbers inline using [Reference: n] style. "
            f"Do not list references at the end. "
        )
    else:
        response_style = """
        STYLE:
        Use a clear, natural, speaker-friendly tone that is short and engaging. No code block or Markdown formatting. 
        One sentence per line. No references at the end.

        FORMATTING:
        After every sentence, insert a newline character (\n).
        Two newline characters (\n\n) indicate a new paragraph.
        Do not join sentences in the same line.
        """
        reference_style = (
            f"Mention specific reference numbers inline when that part of the answer is refer to some reference. "
            f"Do not mention references at the end of the response as user cannot connect them to the answer. "
            f"e.g. According to reference 1, as mention in reference 2, etc. "
        )
    # Create modified message based on whether documents were inserted
    if not insert_document or n == 0:
        modified_message = (
            f"{response_style}"
            f"If the question is a complex question, provide hints, explanations, "
            f"or step-by-step guidance instead of a direct answer. "
            f"If you are unsure after making a reasonable effort, "
            f"explain that there is no data in the knowledge base for the response. "
            f"Only refuse if the question is clearly unrelated to any topic in "
            f"{course}: {class_name} and is not a general, reasonable query. "
            f"If the intent is unclear, ask clarifying questions rather than refusing.\n---\n"
        )
    else:
        modified_message = (
            f"{insert_document}---\n"
            f"{response_style}"
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
            f"If intent is unclear, ask clarifying questions before refusing.\n---\n"
        )
    # Append user instruction to the modified message
    if not practice:
        modified_message += f"Instruction: {user_message}"
    else:
        modified_message += user_message
    # Return the final modified message and reference list
    return modified_message, reference_list
