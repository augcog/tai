"""Internal service to process a directory of files and schedule conversion tasks for each file.
"""
import logging
import os
from concurrent.futures import as_completed
from pathlib import Path
from typing import Dict, Union

from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.conversion.pdf_to_md import PdfToMdConverter
from rag.file_conversion_router.services.task_manager import schedule_conversion

# from rag.file_conversion_router.conversion.text_to_md import TextToMdConverter
# from rag.file_conversion_router.conversion.md_to_md import MdToMdConverter
# from rag.file_conversion_router.conversion.video_to_md import VideoToMdConverter
# from rag.file_conversion_router.conversion.audio_to_md import AudioToMdConverter
# from rag.file_conversion_router.conversion.latex_to_md import LatexToMdConverter

ConverterMapping = Dict[str, BaseConverter]

# Mapping from file extensions to their corresponding conversion classes
converter_mapping: ConverterMapping = {
    '.pdf': PdfToMdConverter(),
    # '.txt': TextToMdConverter(),
    # '.md': MdToMdConverter(),
    # '.mp4': VideoToMdConverter(),
    # '.mp3': AudioToMdConverter(),
    # '.tex': LatexToMdConverter(),
}


def process_folder(input_dir: Union[str, Path], output_dir: Union[str, Path]) -> None:
    """Walk through the input directory and schedule conversion tasks for specified file types.

    Args:
        input_dir (Union[str, Path]): The directory from which to read files.
        output_dir (Union[str, Path]): The directory where converted files will be placed.

    Raises:
        ValueError: If either input_dir or output_dir is not a directory.
    """
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)

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
    for input_file_path in input_dir.rglob('*'):
        if input_file_path.suffix in valid_extensions and input_file_path.is_file():
            # Construct the output subdirectory and file path
            output_subdir = output_dir / input_file_path.relative_to(input_dir).parent
            output_subdir.mkdir(parents=True, exist_ok=True)
            output_file_path = output_subdir / (input_file_path.stem + '.md')

            # Get the converter based on the file extension
            converter = converter_mapping[input_file_path.suffix]
            if converter:
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

    logging.info(f"Completed processing for directory: {input_dir}")
