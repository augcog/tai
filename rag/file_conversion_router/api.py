"""Public API for the file conversion router module."""

from pathlib import Path
from typing import Union
import yaml

from file_conversion_router.services.directory_service import process_folder

def load_yaml(file_path):
    with open(file_path, "r") as file:
        return yaml.safe_load(file)
def convert_directory(input_config) -> None:
    """Convert all supported files in the given directory to Markdown format, to the specified output directory.

    Current supported file types:
    1. PDF
    2. Markdown (To clarify, this markdown includes additional tree structure of original markdown file)
    """
    data = load_yaml(input_config)
    input_dir = data["input_dir"]
    output_dir = data["output_dir"]
    course_name = data["course_name"]
    course_id = data["course_code"]
    log_dir = data.get("log_folder", None)
    cache_path = data.get("cache_path", None)
    process_folder(
        input_dir,
        output_dir,
        course_name,
        course_id,
        log_dir=log_dir,
        cache_path=cache_path,
    )
convert_directory("/home/bot/bot/yk/YK_final/tai/rag/file_conversion_router/config.yaml")