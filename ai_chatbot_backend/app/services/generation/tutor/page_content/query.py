from typing import List
from uuid import UUID

from app.core.models.chat_completion import Message, PageContentParams
from app.services.query.vector_search import get_chunks_by_file_uuid
from app.services.generation.prompts.textchat.page_content import (
    PAGE_CONTENT_WITH_REFS,
    PAGE_CONTENT_NO_REFS,
)


def _resolve_class_name(course_code: str) -> str:
    """Resolve human-readable class name from course code."""
    try:
        from app.services.query.course_mapping import _get_pickle_and_class
        return _get_pickle_and_class(course_code)
    except (ValueError, KeyError):
        return course_code


def build_page_content_context(params: PageContentParams) -> List[Message]:
    """
    Build the [system, user] message pair for page content generation.

    1. For each reference, fetch chunk text from SQLite via get_chunks_by_file_uuid().
    2. Select system prompt (WITH_REFS or NO_REFS).
    3. Build user message with point, purpose, and chunk texts.
    """
    class_name = _resolve_class_name(params.course_code)

    # Fetch chunk texts for each reference
    chunk_texts = []
    for ref in params.references:
        all_chunks = get_chunks_by_file_uuid(UUID(ref.file_uuid))
        for chunk in all_chunks:
            if chunk["index"] == ref.chunk_index:
                chunk_texts.append(chunk["chunk"])
                break

    # Select system prompt variant
    if chunk_texts:
        system_prompt = PAGE_CONTENT_WITH_REFS.format(
            course=params.course_code,
            class_name=class_name,
        )
    else:
        system_prompt = PAGE_CONTENT_NO_REFS.format(
            course=params.course_code,
            class_name=class_name,
        )

    # Build user message
    user_content = f"<point>{params.point}</point>\n\n"
    user_content += f"<purpose>{params.purpose}</purpose>\n\n"

    if chunk_texts:
        user_content += "<reference_materials>\n"
        for i, text in enumerate(chunk_texts, 1):
            user_content += f"--- Reference {i} ---\n{text}\n\n"
        user_content += "</reference_materials>"

    return [
        Message(role="system", content=system_prompt),
        Message(role="user", content=user_content),
    ]
