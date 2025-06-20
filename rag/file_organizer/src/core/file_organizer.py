import os
import shutil
from enum import Enum
from typing import List, Dict
import json

from rag.file_organizer.src.utils.utils import find_markdown_files

from rag.file_organizer.src.utils.logging_service import get_logger
logger = get_logger(__name__)


class ContentCategory(Enum):
    PRACTICE = "practice"
    RESOURCE = "resource"
    STUDY = "study"


def organize_files(input_folder, topics_path, function_path, destination_root: str, move_files: bool = True):

    files = find_markdown_files(input_folder)
    with open(topics_path, 'r') as f:
        topics_dict = json.load(f)
    with open(function_path, 'r') as f:
        functions_dict = json.load(f)

    for file in files:
        file_path = os.path.abspath(file)

        if file_path in topics_dict:
            lecture_topics = topics_dict[file_path]
        else:
            logger.info(f"Skipping {file_path}: not found in topics dictionary")
            lecture_topics = ["unknown"]

        if file_path in functions_dict:
            content_category = functions_dict[file_path]
        else:
            logger.info(f"Skipping {file_path}: not found in functions dictionary")
            content_category = ["unknown"]

        if not lecture_topics:
            logger.info(f"Skipping {file_path}: no topics assigned")
            lecture_topics = ["unknown"]
        if not content_category:
            logger.info(f"Skipping {file_path}: no content category assigned")
            content_category = "unknown"

        # selecting the first topic for simplicity TODO: select most relevant topic
        target_dir = os.path.join(destination_root, lecture_topics[0], content_category)
        os.makedirs(target_dir, exist_ok=True)

        target_path = os.path.join(target_dir, os.path.basename(file_path))

        try:
            if move_files:
                shutil.move(file_path, target_path)
            else:
                shutil.copy2(file_path, target_path)
            print(f"{'Moved' if move_files else 'Copied'} {file_path} -> {target_path}")
        except FileNotFoundError:
            print(f"File not found: {file_path}. Skipping.")
        except Exception as e:
            print(f"Error processing {file_path}: {e}")