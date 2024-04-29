"""Base class for all file type converters.
"""
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Union

from rag.file_conversion_router.utils.logger import conversion_logger, logger
from rag.file_conversion_router.utils.markdown_parser import MarkdownParser
from rag.file_conversion_router.utils.utils import ensure_path


class BaseConverter(ABC):
    """Base class for all file type converters.

    This base class defines the interface for all type converters, with a standardized workflow, which outputs 3 files:
    - Markdown
    - Tree txt
    - Pickle

    All child class need to implement the abstract methods:
    - _to_markdown

    As long as a child class can convert a file to Markdown,
    the base class will handle the rest of the conversion process.
    """

    def __init__(self):
        self._md_parser = None

        self._md_path = None
        self._tree_txt_path = None
        self._pkl_path = None

        self._logger = logger

    @conversion_logger
    def convert(self, input_path: Union[str, Path], output_folder: Union[str, Path]) -> None:
        """Convert an input file to 3 files: Markdown, tree txt, and pkl file, under the output folder.

        Args:
            input_path: The path for a single file to be converted. e.g. 'path/to/file.txt'
            output_folder: The folder where the output files will be saved. e.g. 'path/to/output_folder'
                other files will be saved in the output folder, e.g.:
                - 'path/to/output_folder/file.md'
                - 'path/to/output_folder/file_tree.txt'
                - 'path/to/output_folder/file.pkl'
        """
        input_path, output_folder = ensure_path(input_path), ensure_path(output_folder)
        if not input_path.exists():
            self._logger.error(f"The file {input_path} does not exist.")
            raise FileNotFoundError(f"The file {input_path} does not exist.")

        self._setup_output_paths(input_path, output_folder)

        # This method embeds the abstract method `_to_markdown`, which need to be implemented by the child class.
        self._convert_to_markdown(input_path, self._md_path)
        self._md_parser = MarkdownParser(self._md_path)

        self._convert_md_to_tree_txt_and_pkl(self._md_path, output_folder)

    @conversion_logger
    def _convert_to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format."""
        self._to_markdown(input_path, output_path)

    @conversion_logger
    def _convert_md_to_tree_txt_and_pkl(self, input_path: Path, output_folder: Path) -> None:
        """Convert the input Markdown file to a tree txt file and a pkl file.

        Files will be saved in the same folder as the Markdown filepath set up in the MarkdownParser initialization.
        """
        self._md_parser.concat_print()

    @abstractmethod
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format. To be implemented by subclasses."""
        raise NotImplementedError("This method should be overridden by subclasses.")

    def _setup_output_paths(self, input_path: Union[str, Path], output_folder: Union[str, Path]):
        """Set up the output paths for the Markdown, tree txt, and pkl files."""
        input_path = ensure_path(input_path)
        output_folder = ensure_path(output_folder)
        self._md_path = ensure_path(output_folder / f"{input_path.stem}.md")
        # TODO: below two paths are currently not used
        #  since current MarkdownParser does not support custom output paths.
        self._tree_txt_path = ensure_path(output_folder / f"{input_path.stem}_tree.txt")
        self._pkl_path = ensure_path(output_folder / f"{input_path.stem}.pkl")
