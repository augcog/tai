from typing import Dict, List, Optional, Tuple
from uuid import UUID

from app.services.query.vector_search import get_chunks_by_file_uuid, get_sections_by_file_uuid


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
