from typing import Optional

from pathlib import Path
from rag.file_conversion_router.classes.page import Page  # adjust import to wherever your Page class lives

def process_page_file(
    input_path: str,
    output_pkl: str = None,
    filetype: Optional[str] = None,
    page_url: str = "",
    mapping_json_path: Optional[str] = None
) -> list:
    """
    Read a Markdown (or PDF-derived Markdown) file, split it into chunks via Page,
    and optionally write those chunks to a .pkl file.

    Args:
        input_path (str): Path to the .md (or .txt) file containing your page content.
        output_pkl (str, optional): If provided, dumps the chunks to this pickle path.
        filetype (str, optional): 'md' or 'pdf'.  If None, inferred from input_path suffix.
        page_url (str): Base URL to attach to each chunk.
        mapping_json_path (str, optional): Path to the JSON header-to-page mapping (for PDFs).

    Returns:
        List[Chunk]: The list of Chunk objects produced.
    """
    input_path = Path(input_path)
    # infer filetype
    if filetype is None:
        filetype = input_path.suffix.lstrip('.')
    # load content
    if filetype.lower() in ("md", "markdown", "txt"):
        text = input_path.read_text(encoding="utf-8")
        content = {"text": text}
    else:
        raise ValueError(f"Unsupported filetype '{filetype}' — only Markdown-derived inputs supported here.")

    # prepare mapping path
    mapping_path = Path(mapping_json_path) if mapping_json_path else None

    # instantiate and run
    page = Page(
        pagename=input_path.stem,
        content=content,
        filetype=filetype,
        page_url=page_url,
        mapping_json_path=mapping_path
    )
    page.to_chunk()
    chunks = page.chunks

    # optionally write out a pickle
    if output_pkl:
        page.chunks_to_pkl(output_pkl)

    return chunks
if __name__ == "__main__":
    # Process a Markdown page and pickle the results
    chunks = process_page_file(
        input_path="/Users/yyk956614/tai/rag/file_conversion_router/test_output/fl/61a-fa20-mt1/61a-fa20-mt1.md",
        output_pkl="/Users/yyk956614/tai/rag/file_conversion_router/test_output/fl/61a-fa20-mt1/61a-fa20-mt1.pkl",
        page_url="https://example.com/lecture_notes",
    )
    print(f"Produced {len(chunks)} chunks.")
