"""Internal service to process a directory of files and schedule conversion tasks for each file."""

import logging
# from concurrent.futures import as_completed
from pathlib import Path
from typing import Dict, Type, Union
import os
from typing import List, Tuple
from pathspec import PathSpec
from file_conversion_router.conversion.base_converter import BaseConverter
from file_conversion_router.conversion.ed_converter import EdConverter
from file_conversion_router.conversion.html_converter import HtmlConverter
from file_conversion_router.conversion.md_converter import MarkdownConverter
from file_conversion_router.conversion.notebook_converter import NotebookConverter
from file_conversion_router.conversion.pdf_converter import PdfConverter
from file_conversion_router.conversion.python_converter import PythonConverter
from file_conversion_router.conversion.rst_converter import RstConverter
from file_conversion_router.conversion.video_converter import VideoConverter
from file_conversion_router.services.task_manager import schedule_conversion
from file_conversion_router.utils.conversion_cache import ConversionCache
from file_conversion_router.utils.logger import content_logger, set_log_file_path
from file_conversion_router.utils.utils import load_conversion_version

ConverterMapping = Dict[str, Type[BaseConverter]]

# Mapping from file extensions to their corresponding conversion classes
converter_mapping: ConverterMapping = {
    ".pdf": PdfConverter,
    ".md": MarkdownConverter,
    ".rst": RstConverter,
    ".mp4": VideoConverter,
    ".json": EdConverter,
    ".html": HtmlConverter,
    ".ipynb": NotebookConverter,
    ".py": PythonConverter,
    '.mkv': VideoConverter,
    '.webm': VideoConverter,
    #     TODO: Add more file types and converters here
}


def process_folder(
    input_dir: Union[str, Path],
    output_dir: Union[str, Path],
    course_name: str,
    course_id: str,
    log_dir: Union[str, Path] = None,
    cache_path: Union[str, Path] = None,
) -> None:
    """Walk through the input directory and schedule conversion tasks for specified file types."""
    logging.getLogger().setLevel(logging.INFO)
    output_dir = Path(output_dir)
    input_dir = Path(input_dir)

    if log_dir:
        set_log_file_path(content_logger, log_dir)

    if not input_dir.is_dir():
        raise ValueError(f"Provided input path {input_dir} is not a directory.")

    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)
    elif not output_dir.is_dir():
        raise ValueError(f"Provided output path {output_dir} is not a directory.")

    valid_extensions = tuple(converter_mapping.keys())

    if cache_path:
        ConversionCache.set_cache_path(cache_path)
    else:
        ConversionCache.set_cache_path(None)

    ignore_file = '../file_conversion_router/services/.conversionignore'
    patterns = _load_patterns(ignore_file)
    spec = PathSpec.from_lines("gitwildmatch", patterns)

    def should_ignore(file_path: Path) -> bool:
        """Check if the file path should be ignored based on the patterns."""
        relative_path = str(file_path.relative_to(input_dir))
        should_ignore= spec.match_file(relative_path)
        if should_ignore:
            logging.info(f"Ignoring file: {relative_path} based on ignore patterns.")
        return should_ignore


    # Iterate over all files with specified extensions
    for input_file_path in input_dir.rglob("*"):
        if input_file_path.suffix in valid_extensions and input_file_path.is_file() and not should_ignore(input_file_path):
            # Construct the output subdirectory and file path
            output_subdir = output_dir / input_file_path.relative_to(input_dir).parent
            output_subdir.mkdir(parents=True, exist_ok=True)
            output_file_path = output_subdir / input_file_path.stem
            # output_file_path = output_subdir

            # Instantiate a new converter object for each file based on the file extension
            converter_class = converter_mapping.get(input_file_path.suffix)
            if converter_class:
                converter = converter_class(course_name, course_id)
                converter.convert(input_file_path, output_file_path, input_dir.parent)
                # future = schedule_conversion(
                #     converter.convert, input_file_path, output_file_path, input_dir
                # )
                # futures.append(future)
                logging.info(
                    f"Scheduled conversion for {input_file_path} to {output_file_path}"
                )
            else:
                logging.warning(
                    f"No converter available for file type {input_file_path.suffix}"
                )

    # for future in as_completed(futures):
    #     # try:
    #     result = future.result()
    #     logging.info(f"Conversion result: {result}")
    #     # Handle the successful result here
    #     logging.info("Task completed successfully.")
    #     # except Exception as e:
    #     #     logging.error(f"Conversion failed: {e}")

    content_logger.info(f"Completed content checking for directory: {input_dir}")
    logging.info(f"Completed processing for directory: {input_dir}")
    logging.info(
        f"Saved conversion time [{ConversionCache.calc_total_savings()} seconds] by using cached results."
    )

def _load_patterns(ignore_file= None,
                   extra_patterns= None) -> List[str]:
    with open(ignore_file, "r", encoding="utf-8") as f:
        data = f.read()
    # Keep comments/blank lines—PathSpec handles them.
    return data.splitlines() if ignore_file else [] + (extra_patterns or [])
