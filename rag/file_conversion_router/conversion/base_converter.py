"""Base class for all file type converters.
"""
from abc import ABC, abstractmethod
import logging
from pathlib import Path
from typing import Union

from rag.file_conversion_router.utils.utils import ensure_path

# Configure logging at the module level
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class BaseConverter(ABC):
    """Base class for all file type converters."""

    def __init__(self):
        # Each instance will use the root logger's configuration
        self.logger = logging.getLogger(self.__class__.__name__)

    def convert(self, input_path: Union[str, Path], output_path: Union[str, Path]) -> None:
        """Wrapper method to ensure path conversion before calling the actual convert logic."""
        input_path = ensure_path(input_path)
        output_path = ensure_path(output_path)
        self.perform_conversion(input_path, output_path)

    @abstractmethod
    def perform_conversion(self, input_path: Path, output_path: Path) -> None:
        """Perform the actual conversion. To be implemented by subclasses."""
        raise NotImplementedError("This method should be overridden by subclasses.")
