"""Base class for all file type converters.
"""

from abc import ABC, abstractmethod
from pathlib import Path
from shutil import copy2
from concurrent.futures import Future
from threading import Lock
from typing import Union

from rag.file_conversion_router.utils.logger import conversion_logger, logger
from rag.file_conversion_router.utils.markdown_parser import MarkdownParser
from rag.file_conversion_router.utils.utils import ensure_path, calculate_hash


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
    _cache = {}  # Class-level cache shared across all instances
    _futures_cache = {}  # Class-level cache for futures
    _cache_lock = Lock()  # Lock for thread-safe cache operations

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
                - 'path/to/output_folder/file.md.tree.txt'
                - 'path/to/output_folder/file.md.pkl'
        """
        input_path, output_folder = ensure_path(input_path), ensure_path(output_folder)
        if not input_path.exists():
            self._logger.error(f"The file {input_path} does not exist.")
            raise FileNotFoundError(f"The file {input_path} does not exist.")

        self._setup_output_paths(input_path, output_folder)

        file_hash = calculate_hash(input_path)
        with self._cache_lock:
            if file_hash in self._cache:
                cached_paths = self._cache[file_hash]
                self._logger.info(
                    f"Cached result found, using cached files "
                    f"for input path: {input_path} "
                    f"in output folder: {output_folder}.\n"
                    f"Cached content are: {[str(path) for path in cached_paths]}."
                )
                self._use_cached_files(cached_paths, output_folder)
                return

            if file_hash not in self._futures_cache:
                self._logger.info(f"No future cache found for input path: {input_path}, starting conversion.")
                future = Future()
                self._futures_cache[file_hash] = future
                execute_conversion = True
            else:
                self._logger.info(
                    f"Future cache found for input path: {input_path}, "
                    f"waiting for the conversion to finish."
                )
                future = self._futures_cache[file_hash]
                execute_conversion = False

        if execute_conversion:
            try:
                result = self._convert_and_cache(input_path, output_folder, file_hash)
                future.set_result(result)
            except Exception as e:
                future.set_exception(e)
            finally:
                with self._cache_lock:
                    del self._futures_cache[file_hash]
        else:
            result = future.result()
            self._use_cached_files(result, output_folder)

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

    def _setup_output_paths(self, input_path: Union[str, Path], output_folder: Union[str, Path]):
        """Set up the output paths for the Markdown, tree txt, and pkl files."""
        input_path = ensure_path(input_path)
        output_folder = ensure_path(output_folder)
        self._md_path = ensure_path(output_folder / f"{input_path.stem}.md")
        # TODO: current MarkdownParser does not support custom output paths,
        #  below paths are only used for caching purposes at the moment,
        #  since the markdown parser generates below file paths by default
        self._tree_txt_path = ensure_path(output_folder / f"{input_path.stem}.md.tree.txt")
        self._pkl_path = ensure_path(output_folder / f"{input_path.stem}.md.pkl")

    def _convert_and_cache(self, input_path: Path, output_folder: Path, file_hash: str):
        self._setup_output_paths(input_path, output_folder)
        # This method embeds the abstract method `_to_markdown`, which need to be implemented by the child class.
        self._perform_conversion(input_path, output_folder)
        paths = (self._md_path, self._tree_txt_path, self._pkl_path)
        with self._cache_lock:
            self._cache[file_hash] = paths
        return paths

    def _use_cached_files(self, cached_paths, output_folder):
        """Use cached files and copy them to the specified output folder."""
        output_folder = ensure_path(output_folder)
        output_folder.mkdir(parents=True, exist_ok=True)

        md_path, tree_txt_path, pkl_path = cached_paths
        correct_file_name = self._md_path.stem
        for path, suffix in zip((md_path, tree_txt_path, pkl_path), (".md", ".md.tree.txt", ".md.pkl")):
            des_path = Path(copy2(path, output_folder))
            des_path.rename(output_folder / f"{correct_file_name}{suffix}")

        self._logger.info(f"Copied cached files to {output_folder}.")

    def _perform_conversion(self, input_path: Path, output_folder: Path):
        """Perform the file conversion process."""
        self._convert_to_markdown(input_path, self._md_path)
        self._md_parser = MarkdownParser(self._md_path)
        self._convert_md_to_tree_txt_and_pkl(self._md_path, output_folder)

    @abstractmethod
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format. To be implemented by subclasses."""
        raise NotImplementedError("This method should be overridden by subclasses.")
