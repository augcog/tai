import functools
import logging
import os
from pathlib import Path

from rag.file_conversion_router.utils.time_measure import Timer

format_string = "%(asctime)s - %(levelname)s - %(message)s"
# Configure logging at the module level
logging.basicConfig(level=logging.INFO, format=format_string)
logger = logging.getLogger(__name__)

rag_root = Path(__file__).parent.parent.parent
log_dir = os.path.join(rag_root, 'log')
os.makedirs(log_dir, exist_ok=True)
log_file_path = os.path.join(log_dir, 'content_anomalies.log')

content_logger = logging.getLogger("content_logger")

file_handler = logging.FileHandler(log_file_path, mode='w')
file_handler.setLevel(logging.INFO)
formatter = logging.Formatter(format_string)
file_handler.setFormatter(formatter)
content_logger.addHandler(file_handler)

content_logger.propagate = False
content_logger.info("Content anomalies log file created at path: %s", log_file_path)

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
