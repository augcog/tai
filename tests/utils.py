"""Utility functions to help with testing.
"""

import binascii
import difflib
import logging
from difflib import SequenceMatcher
from pathlib import Path
from typing import List, Union

from colorama import Fore, Style, init

# Initialize colorama
init(autoreset=True)
# Below code uses multiple times of `Style.RESET_ALL` due to some unsolved issue
# but at lease the color now works as expected
logging.basicConfig(
    level=logging.DEBUG,
    format=f"{Fore.WHITE}%(asctime)s - %(levelname)s - %(message)s{Style.RESET_ALL}",
)

SIMILARITY_THRESHOLD = 95


def hex_dump(binary_data: bytes) -> str:
    """Generate a hex dump of binary data."""
    return binascii.hexlify(binary_data).decode("ascii")


def is_binary_file(file_path: Path) -> bool:
    """Determine if a file is binary based on its file extension."""
    binary_extensions = [".pkl", ".bin", ".dat"]
    return file_path.suffix in binary_extensions


def read_file_contents(file_path: Path, binary: bool) -> Union[bytes, str]:
    """Reads file contents as binary or text."""
    mode = "rb" if binary else "r"
    with file_path.open(mode) as file:
        return file.read()


def get_diffs(expected_contents: str, output_contents: str, fromfile: str, tofile: str) -> List[str]:
    """Generates and returns a list of differences between two sets of text, including file names."""
    return list(
        difflib.unified_diff(
            expected_contents.splitlines(keepends=True),
            output_contents.splitlines(keepends=True),
            fromfile=fromfile,
            tofile=tofile,
            lineterm="",
        )
    )


def format_diff_line(line: str) -> str:
    """Applies color formatting to diff output lines."""
    if line.startswith("+"):
        return Fore.GREEN + line
    elif line.startswith("-"):
        return Fore.RED + line
    elif line.startswith("@@"):
        return Fore.CYAN + line
    return line


def format_and_print_diff(differences: List[str], fromfile: str, tofile: str) -> None:
    """Formats and prints differences between two files, specifying the file names."""
    if not differences:
        logging.info(Fore.GREEN + f"No differences found between {fromfile} and {tofile}." + Style.RESET_ALL)
    else:
        logging.info(Fore.RED + f"Differences found between {fromfile} and {tofile}:\n" + Style.RESET_ALL)
        for line in differences:
            formatted_line = format_diff_line(line)
            logging.info(formatted_line + Style.RESET_ALL)


def compare_files(expected_path: Path, output_path: Path, similarity_threshold: int = SIMILARITY_THRESHOLD) -> bool:
    """
    Compares two files based on their contents with a specified similarity threshold.
    If the similarity percentage is below the threshold, differences are shown, and the function returns False.

    Args:
        expected_path: Path to the expected file.
        output_path: Path to the output file.
        similarity_threshold: The minimum similarity percentage required to consider files as matching.

    Returns:
        bool: True if files are considered similar above the threshold, False otherwise.
    """
    binary = is_binary_file(expected_path) or is_binary_file(output_path)
    expected_contents = read_file_contents(expected_path, binary)
    output_contents = read_file_contents(output_path, binary)

    fromfile = str(expected_path)
    tofile = str(output_path)

    if binary:
        hex_expected = hex_dump(expected_contents)
        hex_output = hex_dump(output_contents)
        matcher = SequenceMatcher(None, hex_expected, hex_output)
    else:
        matcher = SequenceMatcher(None, expected_contents, output_contents)

    similarity_percentage = matcher.ratio() * 100

    if similarity_percentage >= similarity_threshold:
        logging.info(Fore.GREEN + f"Files {fromfile} and {tofile} are similar above the threshold ({similarity_percentage:.2f}% similar)." + Style.RESET_ALL)
        return True
    else:
        if binary:
            diffs = get_diffs(hex_expected, hex_output, fromfile, tofile)
        else:
            diffs = get_diffs(expected_contents, output_contents, fromfile, tofile)

        format_and_print_diff(diffs, fromfile, tofile)
        return False


def compare_folders(expected_dir: Path, output_dir: Path, similarity_threshold: int = SIMILARITY_THRESHOLD) -> bool:
    """
    Compares all files within two directories recursively, reports differences,
    and specifies which files are being compared.

    Args:
        expected_dir: The directory containing the expected files.
        output_dir: The directory containing the output files.
        similarity_threshold: The minimum similarity percentage required to consider files as matching.

    Returns:
        bool: True if the folders match, False otherwise.
    """
    expected_files = {file.relative_to(expected_dir) for file in expected_dir.rglob("*") if file.is_file()}
    output_files = {file.relative_to(output_dir) for file in output_dir.rglob("*") if file.is_file()}

    all_matched = True
    # Compare common files
    common_files = expected_files & output_files
    for file in common_files:
        expected_file_path = expected_dir / file
        output_file_path = output_dir / file
        if not compare_files(expected_file_path, output_file_path, similarity_threshold):
            all_matched = False

    # Report extra and missing files
    extra_files = output_files - expected_files
    if extra_files:
        logging.info(Fore.RED + "Extra files in output directory:" + Style.RESET_ALL)
        for file in extra_files:
            logging.info(Fore.RED + str(output_dir / file) + Style.RESET_ALL)
        all_matched = False

    missing_files = expected_files - output_files
    if missing_files:
        logging.info(Fore.YELLOW + "Missing files in expected directory:" + Style.RESET_ALL)
        for file in missing_files:
            logging.info(Fore.YELLOW + str(expected_dir / file) + Style.RESET_ALL)
        all_matched = False

    return all_matched
