"""Utility functions to help with testing.
"""
import difflib
import filecmp
import os
from pathlib import Path
from typing import List, Tuple, Optional, Dict
from colorama import init, Fore, Style
import logging


# Initialize colorama
init(autoreset=True)
# Below code uses multiple times of `Style.RESET_ALL` due to some unsolved issue
# ...but at lease the color now works as expected
logging.basicConfig(level=logging.DEBUG,
                    format=f'{Fore.WHITE}%(asctime)s - %(levelname)s - %(message)s{Style.RESET_ALL}')


def get_diffs(expected_contents: str, output_contents: str, fromfile: str, tofile: str) -> List[str]:
    """Generates and returns a list of differences between two sets of text, including file names."""
    return list(difflib.unified_diff(
        expected_contents.splitlines(keepends=True),
        output_contents.splitlines(keepends=True),
        fromfile=fromfile,
        tofile=tofile,
        lineterm=''
    ))


def format_diff_line(line: str) -> str:
    """Applies color formatting to diff output lines."""
    if line.startswith('+'):
        return Fore.GREEN + line
    elif line.startswith('-'):
        return Fore.RED + line
    elif line.startswith('@@'):
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


def compare_files(expected_path: Path, output_path: Path) -> bool:
    """Compares two files, checks for differences, and prints formatted differences, including file paths."""
    expected_contents = expected_path.read_text()
    output_contents = output_path.read_text()
    fromfile = str(expected_path)
    tofile = str(output_path)
    actual_diffs = get_diffs(expected_contents, output_contents, fromfile, tofile)
    format_and_print_diff(actual_diffs, fromfile, tofile)
    return not actual_diffs  # Return True if no differences


def compare_folders(expected_dir: Path, output_dir: Path) -> bool:
    """
    Compares all files within two directories recursively, reports differences,
    and specifies which files are being compared.
    Args:
        expected_dir (Path): The directory containing the expected files.
        output_dir (Path): The directory containing the output files.
    Returns:
        bool: True if the folders match, False otherwise.
    """
    expected_files = {file.relative_to(expected_dir) for file in expected_dir.rglob('*') if file.is_file()}
    output_files = {file.relative_to(output_dir) for file in output_dir.rglob('*') if file.is_file()}

    all_matched = True
    # Compare common files
    common_files = expected_files & output_files
    for file in common_files:
        expected_file_path = expected_dir / file
        output_file_path = output_dir / file
        if not compare_files(expected_file_path, output_file_path):
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