"""Base classes for all file type converters.
"""
import logging
from abc import ABC, abstractmethod
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from shutil import copy2
from threading import Lock
from typing import Dict, List, Union

import yaml
from rag.file_conversion_router.classes.chunk import Chunk
from rag.file_conversion_router.classes.page import Page
from rag.file_conversion_router.classes.vidpage import VidPage
from rag.file_conversion_router.embedding_optimization.src.pipeline.optimizer import EmbeddingOptimizer
from rag.file_conversion_router.utils.logger import conversion_logger, logger, content_logger
from rag.file_conversion_router.utils.utils import calculate_hash, ensure_path, check_url
from rag.file_conversion_router.utils.conversion_cache import ConversionCache


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
    DEFAULT_EMBEDDING_OPTIMIZATION_CONFIG_PATH = str(
        (Path(__file__).parent / ".." / "embedding_optimization" / "src" / "configs" / "default_config.yaml").resolve())

    def __init__(self, optimizer_config_path: Union[str, Path] = None):
        self._md_parser = None

        self._md_path = None
        self._pkl_path = None

        self._logger = logger
        self._content_logger = content_logger

        self.cache = ConversionCache

        if optimizer_config_path is None:
            optimizer_config_path = self.DEFAULT_EMBEDDING_OPTIMIZATION_CONFIG_PATH
        self.optimizer_config_path = optimizer_config_path

        if optimizer_config_path:
            config_path = Path(optimizer_config_path)
            if not config_path.is_file():
                self._logger.error(f"Optimizer config file does not exist at: {config_path}")
                raise FileNotFoundError(f"Optimizer config file does not exist at: {config_path}")

            self.optimizer = EmbeddingOptimizer(config_path=str(config_path))
            self._logger.info(f"EmbeddingOptimizer initialized with config: {config_path}")
        else:
            self.optimizer = None
            self._logger.info("Embedding optimization is disabled.")

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
        cached_paths = self.cache.get_cached_paths(file_hash)
        same_version = self.cache.version == ConversionCache.get_file_conversion_version(file_hash)
        if cached_paths and all(Path(path).exists() for path in cached_paths) and same_version:
            self._logger.info(
                f"Cached result found, using cached files for input path: {input_path} "
                f"in output folder: {output_folder}."
                f"\n Cached content are: {[str(path) for path in cached_paths]}."
            )
            self._content_logger.warning(f"Cached result found, using cached files for input path: {input_path} ")
            self._use_cached_files(cached_paths, output_folder)
            return

        future = self.cache.get_future(file_hash)
        if not future or not future.running():
            with ThreadPoolExecutor() as executor:
                future = executor.submit(self._convert_and_cache, input_path, output_folder, file_hash)
                self.cache.store_future(file_hash, future)
                self._logger.info("New conversion task started or previous task completed.")
                return

        self._logger.info("Conversion is already in progress, waiting for it to complete.")
        # This will block until the future is completed
        future.result()
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
        self._pkl_path = ensure_path(output_folder / f"{input_path.stem}.pkl")

    def _convert_and_cache(self, input_path: Path, output_folder: Path, file_hash: str) -> List[Path]:
        self._setup_output_paths(input_path, output_folder)
        # This method embeds the abstract method `_to_markdown`, which needs to be implemented by the child classes.
        _, conversion_time = self._perform_conversion(input_path, output_folder)
        paths = [self._md_path, self._pkl_path]
        assert all(path.exists() for path in paths), "Not all output files were generated."
        self.cache.set_cache_and_time(file_hash, str(input_path), paths, conversion_time)
        logger.info(f"cached into {self.cache._cache_file_path}")

    def _use_cached_files(self, cached_paths: List[Path], output_folder: Path) -> None:
        """Use cached files and copy them to the specified output folder, avoiding self-copying."""
        output_folder = ensure_path(output_folder)
        output_folder.mkdir(parents=True, exist_ok=True)

        md_path, pkl_path = cached_paths
        correct_file_name = self._md_path.stem

        for path, suffix in zip((md_path, pkl_path), (".md", ".pkl")):
            des_path = output_folder / f"{correct_file_name}{suffix}"
            path = Path(path)
            # Prevent self-copying if source and destination are identical
            if path.resolve() == des_path.resolve():
                self._logger.info(f"Skipping self-copy: {path} is already in {output_folder}.")
                continue

            des_path = Path(copy2(path, output_folder))
            des_path.rename(output_folder / f"{correct_file_name}{suffix}")
            self._logger.info(f"Copied cached file from {path} to {des_path}.")

    def _read_metadata(self, metadata_path: Path) -> dict:
        """Read metadata from file or return mocked data if file doesn't exist."""
        if metadata_path.exists():
            try:
                with open(metadata_path, "r") as metadata_file:
                    return yaml.safe_load(metadata_file)
            except Exception as e:
                self._logger.error(f"Error reading metadata file: {str(e)}")
                return self._get_mocked_metadata()
        else:
            self._logger.warning(f"Metadata file not found: {metadata_path}. Using mocked metadata.")
            return self._get_mocked_metadata()

    @staticmethod
    def _get_mocked_metadata() -> dict:
        """Return mocked metadata when the actual metadata file is missing."""
        return {
            "URL": "",
            # Add other mocked metadata fields as needed
        }

    @conversion_logger
    def _perform_conversion(self, input_path: Path, output_folder: Path) -> None:
        """Perform the file conversion process."""
        logging.getLogger().setLevel(logging.INFO)

        logger.info(f"🚀 Starting conversion for {input_path}")
        if not output_folder.exists():
            output_folder.mkdir(parents=True, exist_ok=True)
            logger.warning(f"Output folder did not exist, it's now created: {output_folder}")
        filename = output_folder.stem
        pkl_output_path = output_folder / f"{filename}.pkl"
        logger.info(f"📄 Expected Markdown Path: {self._md_path}")
        logger.info(f"🛠️ Expected Pickle Path: {pkl_output_path}")
        try:
            page = self._convert_to_page(input_path, pkl_output_path)
            logger.info("✅ Page conversion successful.")
            page.to_chunk()
            logger.info("✅ Successfully converted page content to chunks.")
        except Exception as e:
            logger.error(f"❌ ERROR during processing: {e}", exc_info=True)


        # Add embedding optimization if enabled
        if self.optimizer:
            # Handle Markdown Optimization
            original_content = page.content.get('text', '')
            self._optimize_markdown_content(page, original_content)

            # Handle Chunk Optimization
            combined_chunks = self._optimize_chunks(page.chunks)
            page.chunks = combined_chunks

        if self._check_page_content(page, input_path):
            logger.info(f"📝 Saving Pickle to {pkl_output_path}")
            page.chunks_to_pkl(str(pkl_output_path))

    def _optimize_markdown_content(self, page: Page, original_content: str) -> None:
        """Optimize the Markdown content and combine enhanced and original versions."""
        result = self.optimizer.process_markdown(original_content)
        if result.success:
            enhanced_content = result.content
            # Combine enhanced and original content with clear headers
            """Uncomment below line after have way to deactivate optimizer"""
            combined_content = (
                # "# TAI Embedding Optimized Content\n\n"
                # f"{enhanced_content}\n\n"
                # "# Original Content\n\n"
                f"{original_content}"
            )
            # Update the page content with combined content
            page.content['text'] = combined_content

            # Resave the combined Markdown content
            with open(self._md_path, "w", encoding="utf-8") as md_file:
                md_file.write(combined_content)
            self._logger.info(f"Enhanced and original Markdown saved to {self._md_path}")
        else:
            self._logger.error(f"Failed to optimize Markdown: {result.error}")

    def _optimize_chunks(self, original_chunks: List[Chunk]) -> List[Chunk]:
        """Optimize each chunk and combine enhanced and original versions."""
        optimized_chunks = self.optimizer.process_chunks(original_chunks)
        combined_chunks = []

        for original_chunk, optimized_chunk in zip(original_chunks, optimized_chunks):
            # Combine enhanced and original chunk content with clear headers
            """Uncomment below line after have way to deactivate optimizer"""
            combined_chunk_content = (
                # "## TAI Embedding Optimized Chunk\n\n"
                # f"{optimized_chunk.content}\n\n"
                # "## Original Chunk\n\n"
                f"{original_chunk.content}"
            )

            # Create a new Chunk instance with combined content
            combined_chunk = Chunk(
                content=combined_chunk_content,
                titles=original_chunk.titles,
                chunk_url=original_chunk.chunk_url,
                metadata={
                    **(original_chunk.metadata or {}),
                    'enhanced': True,
                    'original_chunk_url': original_chunk.chunk_url  # Preserve original URL if needed
                },
                page_num = original_chunk.page_num if original_chunk.page_num else None
            )
            combined_chunks.append(combined_chunk)

            # self._logger.info(f"Combined enhanced and original chunk for URL: {original_chunk.page_num}")

        return combined_chunks



    def _check_page_content(self, page: Page, input_path: Path) -> bool:
        content_length_threshold = Page.PAGE_LENGTH_THRESHOLD
        content_length = len(page.content.get('text', ''))
        page_url = page.page_url
        url_state = check_url(page_url)
        if content_length < content_length_threshold:
            self._content_logger.warning(f"File: {input_path} removed. Page has content length: {content_length} "
                                         f"less than threshold: {content_length_threshold}")
            return False

        if url_state != 200:
            self._content_logger.error(f"File: {input_path} has url state: {url_state}")
        else:
            self._content_logger.info(f"File: {input_path} has content length {content_length}, "
                                      f"url state: {url_state}")
        return True


    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        # Ensure the output directory exists
        output_path.parent.mkdir(parents=True, exist_ok=True)
        stem = input_path.stem
        file_type = input_path.suffix.lstrip('.')

        md_path = self._to_markdown(input_path, output_path)
        with open(md_path, "r") as input_file:
            content_text = input_file.read()

        metadata_path = input_path.with_name(f"{input_path.stem}_metadata.yaml")

        page_path = output_path.with_name(f"{stem}_page_info.yaml")


        metadata_content = self._read_metadata(metadata_path)
        url = metadata_content.get("URL")

        if file_type == "mp4":
            timestamp = [i[1] for i in self.paragraphs]
            content = {"text": content_text, "timestamp": timestamp}
            return VidPage(pagename=stem, content=content, filetype=file_type, page_url=url)
        else:
            content = {"text": content_text}
            return Page(pagename=stem, content=content, filetype=file_type, page_url=url, page_path=page_path)

    @abstractmethod
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format. To be implemented by subclasses."""
        raise NotImplementedError("This method should be overridden by subclasses.")
