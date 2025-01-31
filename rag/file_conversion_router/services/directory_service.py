"""Internal service to process a directory of files and schedule conversion tasks for each file.
"""

import logging
from concurrent.futures import as_completed
from pathlib import Path
from typing import Dict, Type, Union

from rag.file_conversion_router.conversion.base_converter import BaseConverter, ConversionCache
from rag.file_conversion_router.conversion.md_converter import MarkdownConverter
from rag.file_conversion_router.conversion.pdf_converter import PdfConverter
from rag.file_conversion_router.conversion.rst_converter import RstConverter
from rag.file_conversion_router.conversion.video_converter import VideoConverter
from rag.file_conversion_router.conversion.ed_converter import EdConverter
from rag.file_conversion_router.conversion.html_converter import HtmlConverter
from rag.file_conversion_router.services.task_manager import schedule_conversion
from rag.file_conversion_router.utils.logger import content_logger, set_log_file_path
from rag.file_conversion_router.utils.utils import calculate_hash
from rag.file_conversion_router.utils.persistent_cache import *
from rag.file_conversion_router.utils.utils import load_conversion_version, is_empty_md


ConverterMapping = Dict[str, Type[BaseConverter]]

# Mapping from file extensions to their corresponding conversion classes
converter_mapping: ConverterMapping = {
    ".pdf": PdfConverter,
    ".md": MarkdownConverter,
    ".rst": RstConverter,
    ".mp4": VideoConverter,
    ".json": EdConverter,
    ".html": HtmlConverter
    #     TODO: Add more file types and converters here
}


def process_folder(input_dir: Union[str, Path], output_dir: Union[str, Path],
                   log_dir: Union[str, Path] = None, cache_dir: Union[str, Path] = None) -> None:
    """Walk through the input directory and schedule conversion tasks for specified file types.

    Args:
        input_dir (Union[str, Path]): The directory from which to read files.
        output_dir (Union[str, Path]): The directory where converted files will be placed.
        log_dir (Union[str, Path], optional): The directory where log files will be placed. Defaults to None.
        cache_dir (Union[str, Path], optional): The directory where cache files will be placed. Defaults to None.
    Raises:
        ValueError: If either input_dir or output_dir is not a directory.
    """
    output_dir = Path(output_dir)
    input_dir = Path(input_dir)

    cache_file_name = "persistent_cache.pkl"
    persistent_cache = {}
    if log_dir:
        set_log_file_path(content_logger, log_dir)
    if cache_dir:
        cache_path = Path(cache_dir) / cache_file_name
        persistent_cache = load_persistent_cache(cache_path)

    version_file_path = Path(__file__).parent.parent / "conversion_version.txt"
    conversion_version = load_conversion_version(version_file_path)

    if not input_dir.is_dir():
        raise ValueError(f"Provided input path {input_dir} is not a directory.")

    # Validate that output_dir is a directory; create if it does not exist
    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)
    elif not output_dir.is_dir():
        raise ValueError(f"Provided output path {output_dir} is not a directory.")

    # Predefined file extensions to look for, based on the converter mapping
    valid_extensions = tuple(converter_mapping.keys())
    futures = []

    # Iterate over all files with specified extensions
    for input_file_path in input_dir.rglob("*"):
        if input_file_path.suffix in valid_extensions and input_file_path.is_file():
            if cache_dir:
                file_hash = calculate_hash(input_file_path)
                if is_empty_md(file_hash):
                    content_logger.error(f"conversion skipped: found empty markdown file {input_file_path}")
                    continue
                cached_result = persistent_cache.get(file_hash, None)
                if cached_result and cached_result == conversion_version:
                    content_logger.info(f"Using persistent cached result version {conversion_version} for {input_file_path}")
                    continue
                else:
                    persistent_cache[file_hash] = conversion_version

            # Construct the output subdirectory and file path
            output_subdir = output_dir / input_file_path.relative_to(input_dir).parent
            output_subdir.mkdir(parents=True, exist_ok=True)
            output_file_path = output_subdir / input_file_path.stem
            # output_file_path = output_subdir

            # Instantiate a new converter object for each file based on the file extension
            converter_class = converter_mapping.get(input_file_path.suffix)
            if converter_class:
                converter = converter_class()
                future = schedule_conversion(converter.convert, input_file_path, output_file_path)
                futures.append(future)
                logging.info(f"Scheduled conversion for {input_file_path} to {output_file_path}")
            else:
                logging.warning(f"No converter available for file type {input_file_path.suffix}")

    for future in as_completed(futures):
        try:
            result = future.result()
            logging.info(f"Conversion result: {result}")
            # Handle the successful result here
            logging.info("Task completed successfully.")
        except Exception as e:
            logging.error(f"Conversion failed: {e}")

    if cache_dir:
        cache_dir = Path(cache_dir)
        os.makedirs(cache_dir, exist_ok=True)
        save_persistent_cache(persistent_cache, cache_dir / cache_file_name)
    content_logger.info(f"Completed content checking for directory: {input_dir}")
    logging.info(f"Completed processing for directory: {input_dir}")
    logging.info(f"Saved conversion time [{ConversionCache.calc_total_savings()} seconds] by using cached results.")
