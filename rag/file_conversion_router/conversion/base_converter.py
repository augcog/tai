"""Base classes for all file type converters.
"""

from abc import ABC, abstractmethod
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from shutil import copy2
from threading import Lock
from typing import Dict, List, Union

from rag.file_conversion_router.utils.logger import conversion_logger, logger
from rag.file_conversion_router.utils.markdown_parser import MarkdownParser
from rag.file_conversion_router.utils.utils import calculate_hash, ensure_path
from rag.file_conversion_router.classes.page import Page


class BaseConverter(ABC):
    """Base classes for all file type converters.

    This base classes defines the interface for all type converters, with a standardized workflow, which outputs 3 files:
    - Markdown
    - Tree txt
    - Pickle

    All child classes need to implement the abstract methods:
    - _to_markdown

    As long as a child classes can convert a file to Markdown,
    the base classes will handle the rest of the conversion process.
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
                - 'path/to/output_folder/file.md.tree.txt'
                - 'path/to/output_folder/file.md.pkl'
        """
        input_path, output_folder = ensure_path(input_path), ensure_path(output_folder)
        if not input_path.exists():
            self._logger.error(f"The file {input_path} does not exist.")
            raise FileNotFoundError(f"The file {input_path} does not exist.")

        self._setup_output_paths(input_path, output_folder)

        file_hash = calculate_hash(input_path)
        cached_paths = ConversionCache.get_cached_paths(file_hash)
        if cached_paths:
            self._logger.info(
                f"Cached result found, using cached files for input path: {input_path} "
                f"in output folder: {output_folder}."
                f"\n Cached content are: {[str(path) for path in cached_paths]}."
            )
            self._use_cached_files(cached_paths, output_folder)
            return

        future = ConversionCache.get_future(file_hash)
        if not future or not future.running():
            with ThreadPoolExecutor() as executor:
                future = executor.submit(self._convert_and_cache, input_path, output_folder, file_hash)
                ConversionCache.store_future(file_hash, future)
                self._logger.info("New conversion task started or previous task completed.")
                return

        self._logger.info("Conversion is already in progress, waiting for it to complete.")
        # This will block until the future is completed
        future.result()
        cached_paths = ConversionCache.get_cached_paths(file_hash)
        self._logger.info(
            f"Future completed, using cached files for input path: {input_path} "
            f"in output folder: {output_folder}."
            f"\n Cached content are: {[str(path) for path in cached_paths]}."
        )
        self._use_cached_files(cached_paths, output_folder)

    @conversion_logger
    def _convert_to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format."""
        self._to_markdown(input_path, output_path)

    @conversion_logger
    def _convert_to_page(self, input_path: Path, output_path: Path) -> Page:
        page = self._to_page(input_path, output_path)
        return page

    def _setup_output_paths(self, input_path: Union[str, Path], output_folder: Union[str, Path]) -> None:
        """Set up the output paths for the Markdown, tree txt, and pkl files."""
        input_path = ensure_path(input_path)
        output_folder = ensure_path(output_folder)
        self._md_path = ensure_path(output_folder / f"{input_path.stem}.md")
        # TODO: current MarkdownParser does not support custom output paths,
        #  below paths are only used for caching purposes at the moment,
        #  since the markdown parser generates below file paths by default
        self._tree_txt_path = ensure_path(output_folder / f"{input_path.stem}.md.tree.txt")
        self._pkl_path = ensure_path(output_folder / f"{input_path.stem}.md.pkl")

    def _convert_and_cache(self, input_path: Path, output_folder: Path, file_hash: str) -> List[Path]:
        self._setup_output_paths(input_path, output_folder)
        # This method embeds the abstract method `_to_markdown`, which needs to be implemented by the child classes.
        _, conversion_time = self._perform_conversion(input_path, output_folder)
        paths = [self._md_path, self._tree_txt_path, self._pkl_path]
        ConversionCache.set_cached_paths_and_time(file_hash, paths, conversion_time)

    def _use_cached_files(self, cached_paths: List[Path], output_folder: Path) -> None:
        """Use cached files and copy them to the specified output folder."""
        output_folder = ensure_path(output_folder)
        output_folder.mkdir(parents=True, exist_ok=True)

        md_path, tree_txt_path, pkl_path = cached_paths
        correct_file_name = self._md_path.stem
        for path, suffix in zip((md_path, tree_txt_path, pkl_path), (".md", ".md.tree.txt", ".md.pkl")):
            des_path = Path(copy2(path, output_folder))
            des_path.rename(output_folder / f"{correct_file_name}{suffix}")
            self._logger.info(f"Copied cached file from {path} to {des_path}.")

    @conversion_logger
    def _perform_conversion(self, input_path: Path, output_folder: Path) -> None:
        """Perform the file conversion process."""
        page = self._convert_to_page(input_path, output_folder)[0]
        page.to_chunk()
        pkl_file = output_folder.with_suffix(".pkl")
        page.chunks_to_pkl(str(pkl_file))

    @abstractmethod
    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        """Convert the input file to Expected Page format. To be implemented by subclasses."""
        raise NotImplementedError("This method should be overridden by subclasses.")


class ConversionCache:
    """A classes to handle caching of conversion results."""
    _cache: Dict[str, List[Path]] = {}
    _futures_cache: Dict[str, Future] = {}
    # Store the time taken for each file conversion
    _times_cache: Dict[str, float] = {}
    # Store the frequency of access to each cache item
    _access_count: Dict[str, int] = {}
    _lock = Lock()

    @classmethod
    def get_cached_paths(cls, file_hash: str) -> Union[List[Path], None]:
        with cls._lock:
            if file_hash in cls._cache:
                cls._access_count[file_hash] = cls._access_count.get(file_hash, 0) + 1
            return cls._cache.get(file_hash)

    @classmethod
    def set_cached_paths_and_time(cls, file_hash: str, paths: List[Path], time_taken: float):
        with cls._lock:
            cls._cache[file_hash] = paths
            cls._times_cache[file_hash] = time_taken
            cls._access_count[file_hash] = 0

    @classmethod
    def get_cached_time(cls, file_hash: str) -> float:
        with cls._lock:
            return cls._times_cache.get(file_hash, None)

    @classmethod
    def get_access_count(cls, file_hash: str) -> int:
        with cls._lock:
            return cls._access_count.get(file_hash, 0)

    @classmethod
    def get_future(cls, file_hash: str) -> Future:
        with cls._lock:
            return cls._futures_cache.get(file_hash)

    @classmethod
    def store_future(cls, file_hash: str, future: Future) -> None:
        with cls._lock:
            cls._futures_cache[file_hash] = future

    @classmethod
    def clear_future(cls, file_hash: str) -> None:
        with cls._lock:
            del cls._futures_cache[file_hash]

    @classmethod
    def calc_total_savings(cls) -> float:
        """Calculate total time saved by using cached results based on access frequency and initial conversion time."""
        with cls._lock:
            return sum(
                time * (accesses - 1) for file_hash, time in cls._times_cache.items()
                if (accesses := cls._access_count.get(file_hash, 0)) > 1
            )
