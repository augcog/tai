"""Public API for the file conversion router module.
"""

from pathlib import Path
from typing import Union
import sys

# Add the parent directory to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from rag.file_conversion_router.services.directory_service import process_folder

def convert_directory(input_dir: Union[str, Path], output_dir: Union[str, Path],
                      log_dir: Union[str, Path] = None, cache_dir: Union[str, Path] = None) -> None:
    """Convert all supported files in the given directory to Markdown format, to the specified output directory.

    Current supported file types:
    1. PDF
    2. Markdown (To clarify, this markdown includes additional tree structure of original markdown file)
    """
    process_folder(input_dir, output_dir, log_dir=log_dir, cache_dir=cache_dir)

convert_directory(r'C:\Users\altai\Desktop\projects\roar_tai\tai\roar_test_conversion\input', r'C:\Users\altai\Desktop\projects\roar_tai\tai\roar_test_conversion\output')
# convert_directory(r'C:\Users\altai\Desktop\projects\roar_tai\tai\roar_test_conversion\input\1-1-introduction-to-python-programming.ipynb', r'C:\Users\altai\Desktop\projects\roar_tai\tai\roar_test_conversion\output')
