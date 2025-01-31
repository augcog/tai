import os
import logging
from pathlib import Path

logger = logging.getLogger('scraper')
logger.setLevel(logging.INFO)

formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')


def set_up_logger(logger, output_path, mode='w'):
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)

    os.makedirs(Path(output_path).parent, exist_ok=True)

    try:
        file_handler = logging.FileHandler(output_path, mode=mode)
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
    except Exception as e:
        logger.error(f"Failed to set content logger file path: {e}")