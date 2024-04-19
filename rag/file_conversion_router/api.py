"""Public API for the file conversion router module.
"""
from services.directory_service import process_folder


def convert_directory(input_dir: str, output_dir: str):
    """Convert all supported files in the given directory to Markdown format, to the specified output directory.

    Current supported file types:
    1. PDF
    2. Markdown (To clarify, this markdown includes additional tree structure of original markdown file)
    """
    process_folder(input_dir, output_dir)
