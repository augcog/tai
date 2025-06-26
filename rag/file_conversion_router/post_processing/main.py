from dotenv import load_dotenv
from rag.file_conversion_router.post_processing.MarkdownProcessor.BaseMarkdownProcessor import MarkdownStructureBase
from rag.file_conversion_router.post_processing.MarkdownProcessor.PDFMarkdwonStructure import PdfMarkdownStructure
from rag.file_conversion_router.post_processing.MarkdownProcessor.VideoMarkdownProcessor import VideoMarkdownStructure
import os
from pathlib import Path
from typing import Literal, Union



def create_structure_processor(
        api_key: str,
        file_path: Union[str, Path],
        file_type: Literal["pdf", "mp4", "mp3"],
        course_name: str
) -> MarkdownStructureBase:
    """
    Factory function that returns the appropriate markdown structuring processor
    based on the file type.
    """
    file_type = file_type.lower()
    if file_type == "pdf":
        return PdfMarkdownStructure(api_key, file_path, course_name)
    elif file_type in ("mp4", "mp3"):
        return VideoMarkdownStructure(api_key, file_path, course_name)
    else:
        raise ValueError(f"Unsupported file type: '{file_type}'. Supported types are 'pdf', 'mp4', 'mp3'.")


if __name__ == "__main__":
    # Best practice: Load API key from environment variable
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise ValueError("OPENAI_API_KEY environment variable not set. Please set it before running the script.")

    # --- Configuration ---
    course_name_61a = "Structure and Interpretation of Computer Programs"
    course_name_294 = "Immersive Computing and Virtual Reality"

    # Paths are relative to this main.py file
    file_path_61a = Path("/Users/yyk956614/tai/rag/test_output/30-Programs_as_Data_1pp.md")
    file_path_294 = Path("/Users/yyk956614/tai/rag/test_output/Interaction Techniques Part 3 - Symbolic Input and Text Entry.md")

    # --- Example 1: Processing a PDF-derived markdown file ---
    try:
        print("\n>>> PROCESSING PDF MARKDOWN <<<")
        pdf_processor = create_structure_processor(
            api_key=api_key,
            file_path=file_path_61a,
            file_type="pdf",
            course_name=course_name_61a
        )
        pdf_output_path = pdf_processor.structure_file()
        print(f"Final PDF output at: {pdf_output_path}")
    except (FileNotFoundError, ValueError, KeyError, IndexError) as e:
        print(f"\nERROR processing PDF file: {e}\n")

    # # --- Example 2: Processing a Video transcript markdown file ---
    try:
        print("\n>>> PROCESSING VIDEO MARKDOWN <<<")
        video_processor = create_structure_processor(
            api_key=api_key,
            file_path=file_path_294,
            file_type="mp4",
            course_name=course_name_294
        )
        video_output_path = video_processor.structure_file()
        print(f"Final Video output at: {video_output_path}")
    except (FileNotFoundError, ValueError, KeyError, IndexError) as e:
        print(f"\nERROR processing Video file: {e}\n")