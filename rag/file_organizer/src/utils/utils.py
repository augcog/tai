from typing import List
import os
import glob
import json
from typing import Dict, Any


def save_dict_to_json(summaries: Dict, output_path: str) -> None:
    """
    Save summaries to a JSON file.

    Args:
        summaries: Dictionary mapping file paths to summaries
        output_path: Path where to save the JSON file

    Raises:
        ValueError: If the output directory doesn't exist
        IOError: If there's an error writing the file
    """
    # Convert to list of dictionaries for better JSON structure
    # Ensure output directory exists
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(summaries, f, indent=2, ensure_ascii=False)
    except Exception as e:
        raise IOError(f"Error saving summaries to {output_path}: {str(e)}")


def find_markdown_files(folder_path: str, text_filter: str = None) -> List[str]:
    """
    Find all markdown files in a directory and its subdirectories.

    Args:
        folder_path: Path to the root directory to search
        text_filter: Optional text to filter filenames

    Returns:
        List of paths to markdown files
    """
    if not os.path.isdir(folder_path):
        raise ValueError(f"The folder path '{folder_path}' does not exist.")

    # Use glob to find all .md files recursively
    file_pattern = os.path.join(folder_path, "**/*.md")
    files = glob.glob(file_pattern, recursive=True)

    # Apply text filter if provided
    if text_filter:
        files = [file for file in files if text_filter in os.path.basename(file)]

    return files

def all_subfolders_non_empty(root_dir: str) -> bool:
    """
    Check that all immediate subfolders of `root_dir` are non-empty.

    Args:
        root_dir (str): Path to the root directory

    Returns:
        bool: True if all subfolders are non-empty, False otherwise
    """
    for name in os.listdir(root_dir):
        subfolder_path = os.path.join(root_dir, name)
        if os.path.isdir(subfolder_path):
            if not any(os.listdir(subfolder_path)):
                return False
    return True

def split_into_chunks(text: str, max_chunk_size: int = 2500) -> List[str]:
    """Split text into smaller chunks of a specified maximum size."""
    lines = text.splitlines()
    chunks = []
    current_chunk = []

    for line in lines:
        if len(" ".join(current_chunk + [line])) <= max_chunk_size:
            current_chunk.append(line)
        else:
            chunks.append("\n".join(current_chunk))
            current_chunk = [line]

    if current_chunk:
        chunks.append("\n".join(current_chunk))

    return chunks