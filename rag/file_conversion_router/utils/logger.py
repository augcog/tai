import functools
import logging
import os
from pathlib import Path
import threading

from file_conversion_router.utils.time_measure import Timer

format_string = "%(asctime)s - %(levelname)s - %(message)s"
# Configure logging at the module level
logging.basicConfig(level=logging.INFO, format=format_string)
logger = logging.getLogger(__name__)

content_logger = logging.getLogger("content_logger")
content_logger.setLevel(logging.INFO)
content_logger.propagate = False


def set_log_file_path(logger, output_path, mode="w"):
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)

    os.makedirs(output_path, exist_ok=True)

    # Configure the log file
    log_file_path = os.path.join(output_path, f"{logger.name}.log")
    try:
        file_handler = logging.FileHandler(log_file_path, mode=mode)
        file_handler.setFormatter(logging.Formatter(format_string))
        logger.addHandler(file_handler)
    except Exception as e:
        logger.error(f"Failed to set content logger file path: {e}")
        raise

    return log_file_path


def conversion_logger(method):
    """Decorator to log the beginning and end of conversions, including timing.

    Example Output:
    ```
    2021-09-30 15:00:00,000 - INFO - Starting _to_markdown for path/to/file.txt on CPU
    2021-09-30 15:00:10,000 - INFO - [10.00 seconds] Successfully executed _to_markdown for path/to/file.txt to path/to/file.md
    ```
    """

    @functools.wraps(method)
    def wrapper(self, input_path: Path, output_path: Path, *args, **kwargs):
        logger.info(f"Starting {method.__name__} for {input_path}")

        with Timer() as timer:
            result = method(self, input_path, output_path, *args, **kwargs)

        logger.info(
            f"[{timer.interval:.2f} seconds] Successfully executed {method.__name__} "
            f"for {input_path} to {output_path}"
        )
        return result, timer.interval

    return wrapper
