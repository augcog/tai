from typing import Any

from app.config import settings
from app.dependencies.model import get_vllm_chat_client

# Query reformulator prompt
_QUERY_REFORMULATOR_PROMPT = (
    "You are a query reformulator for a RAG system. "
    "Given the user message and the memory synopsis of the current conversation as well as the file context if any, "
    "rewrite the latest user request as a single, "
    "self-contained question for document retrieval. "
    "Resolve pronouns and references using context. "
    "If a list of available course materials is provided, use their descriptions "
    "to align terminology and target specific topics. Include relevant constraints "
    "(dates, versions, scope), and avoid adding facts not in the history. "
    "Return only the rewritten query as question in plain text—no quotes, no extra text."
    "# Valid channels: analysis, commentary, final. Channel must be included for every message."
    "Calls to these tools must go to the commentary channel: 'functions'.<|end|>"
)


async def build_retrieval_query(
    user_message: str,
    memory_synopsis: Any,
    file_sections: Any = None,
    excerpt: Any = None,
    course_descriptions: Any = None,
) -> str:
    """
    Reformulate the latest user request into a single self-contained query string,
    based on the full chat history (user + assistant messages).
    Always uses the local vLLM model regardless of the main LLM mode.
    Returns plain text with no quotes or extra formatting.
    """
    system_prompt = _QUERY_REFORMULATOR_PROMPT

    # If no context is provided, return the original user message
    if not memory_synopsis and not file_sections and not excerpt and not course_descriptions:
        return user_message

    request_parts = []

    if memory_synopsis:
        request_parts.append(f"Memory Synopsis:\n{memory_synopsis}\n")

    if course_descriptions:
        desc_lines = "\n".join(
            f"- {d['file_name']}: {d['description']}"
            for d in course_descriptions
        )
        request_parts.append(f"Available Course Materials:\n{desc_lines}\n")

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

    print(f"[DEBUG] Reformulation input ({len(request_content)} chars):\n{request_content[:2000]}...")

    client = get_vllm_chat_client()
    response = await client.chat.completions.create(
        model=settings.vllm_chat_model,
        messages=chat,
        temperature=0.6,
        top_p=0.95,
        max_tokens=500,
        extra_body={"top_k": 20, "min_p": 0}
    )
    # vLLM with --reasoning-parser separates reasoning_content from content
    # Use content directly (final response without thinking)
    text = response.choices[0].message.content or ""
    print(f"[INFO] Generated RAG-Query: {text.strip()}")
    return text.strip()
