"""Task manager module for scheduling file conversion tasks.
"""
import logging
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Callable, Union

executor = ThreadPoolExecutor(max_workers=10)

ConverterFunc = Callable[[str, str], Any]


def schedule_conversion(converter_func: ConverterFunc,
                        input_path: Union[str, Path], output_path: Union[str, Path]) -> Any:
    """Schedule a file conversion task using a thread pool and return the future.

    Returns:
        Any: The future object representing the conversion task.
    """
    # Submit the conversion task to the thread pool
    return executor.submit(converter_func, input_path, output_path)
