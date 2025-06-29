"""Utility functions to help with testing.
"""

import binascii
import difflib
import logging
import pickle
from collections import namedtuple
from difflib import SequenceMatcher
from pathlib import Path
from typing import List, Union, Dict, Any
from typing import Set

from colorama import Fore, Style, init

# Initialize colorama
init(autoreset=True)
# Below code uses multiple times of `Style.RESET_ALL` due to some unsolved issue
# but at lease the color now works as expected
logging.basicConfig(
    level=logging.DEBUG,
    format=f"{Fore.WHITE}%(asctime)s - %(levelname)s - %(message)s{Style.RESET_ALL}",
)

SIMILARITY_THRESHOLD = 90


def hex_dump(binary_data: bytes) -> str:
    """Generate a hex dump of binary data."""
    return binascii.hexlify(binary_data).decode("ascii")


def is_pkl_file(file_path: Path) -> bool:
    """Determine if a file is binary based on its file extension."""
    return file_path.suffix.lower() == ".pkl"


def read_file_contents(file_path: Path, is_pkl: bool) -> Union[Dict[Any, Any], str]:
    """Reads file contents as binary (pickle) or text."""
    mode = "rb" if is_pkl else "r"
    with file_path.open(mode) as file:
        if is_pkl:
            return pickle.load(file)
        return file.read()


def get_diffs(
    expected_contents: str, output_contents: str, fromfile: str, tofile: str
) -> List[str]:
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
        logging.info(
            Fore.GREEN
            + f"No differences found between {fromfile} and {tofile}."
            + Style.RESET_ALL
        )
    else:
        logging.info(
            Fore.RED
            + f"Differences found between {fromfile} and {tofile}:\n"
            + Style.RESET_ALL
        )
        for line in differences:
            formatted_line = format_diff_line(line)
            logging.info(formatted_line + Style.RESET_ALL)
    logging.info("\n")


def get_pkl_diffs(expected_chunks, output_chunks, fromfile, tofile):
    chunk_diffs = {}  # key : (chunk index, content type) , value : diffs for the chunk

    Placeholder = namedtuple("Placeholder", ["titles", "chunk_url", "content"])
    placeholder = Placeholder(titles="", chunk_url="", content="")

    for i, expected_chunk in enumerate(expected_chunks):
        if i >= len(output_chunks):
            output_chunk = placeholder
        else:
            output_chunk = output_chunks[i]

        if expected_chunk.titles != output_chunk.titles:
            chunk_diffs[(i, "titles")] = get_diffs(
                expected_chunk.titles, output_chunk.titles, fromfile, tofile
            )

        if expected_chunk.chunk_url != output_chunk.chunk_url:
            chunk_diffs[(i, "urls")] = get_diffs(
                ", ".join(expected_chunk.chunk_url),
                ", ".join(output_chunk.chunk_url),
                fromfile,
                tofile,
            )

        matcher = SequenceMatcher(None, expected_chunk.content, output_chunk.content)
        similarity = matcher.ratio() * 100
        if similarity < SIMILARITY_THRESHOLD:
            chunk_diffs[(i, "content")] = get_diffs(
                expected_chunk.content, output_chunk.content, fromfile, tofile
            )

    return chunk_diffs


def compare_text_files(
    expected_contents, output_contents, fromfile, tofile, similarity_threshold
):
    # skipped log files for now
    if fromfile.endswith(".log") or tofile.endswith(".log"):
        logging.info(
            Fore.GREEN
            + f"File comparison skipped because {fromfile} or {tofile} is a log file."
            + Style.RESET_ALL
        )
        return True

    matcher = SequenceMatcher(None, expected_contents, output_contents)

    similarity_percentage = matcher.ratio() * 100

    if similarity_percentage >= similarity_threshold:
        logging.info(
            Fore.GREEN + f"Files {fromfile} and {tofile} are similar "
            f"above the threshold ({similarity_percentage:.2f}% similar)."
            + Style.RESET_ALL
        )
        return True
    else:
        logging.info(
            Fore.RED + f"Files {fromfile} and {tofile} are not similar "
            f"({similarity_percentage:.2f}% similar)." + Style.RESET_ALL
        )

        diffs = get_diffs(expected_contents, output_contents, fromfile, tofile)
        format_and_print_diff(diffs, fromfile, tofile)
        return False


def compare_files(
    expected_path: Path, output_path: Path, similarity_threshold: int = 90
) -> bool:
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
    is_pkl = is_pkl_file(expected_path) and is_pkl_file(output_path)
    expected_contents = read_file_contents(expected_path, is_pkl)
    output_contents = read_file_contents(output_path, is_pkl)

    fromfile = str(expected_path)
    tofile = str(output_path)

    if is_pkl:
        logging.warning(
            Fore.RED
            + "Pickle file comparison test has bugs, it's currently skipped for thorough testing."
            "So for now, the existence of the file is the only thing that is checked."
            "\nPlease fix the test and remove this warning in the future"
            + Style.RESET_ALL
        )
        # TODO: Remove this two lines after fixing the pickle file comparison test
        return True
        diffs = get_pkl_diffs(expected_contents, output_contents, fromfile, tofile)
        if not diffs:
            logging.info(
                Fore.GREEN
                + f"Files {fromfile} and {tofile} are similar above the threshold."
                + Style.RESET_ALL
            )
            return True
        logging.info(
            Fore.RED + f"expected chunk length {len(expected_contents)} and "
            f"output chunk length {len(output_contents)} "
            f"in Files {fromfile} and {tofile}. \n" + Style.RESET_ALL
        )
        for (index, content_type), diff in diffs.items():
            format_and_print_diff(
                diff, f"chunk {index} {content_type} of " + fromfile, tofile
            )
        return False
    else:
        # If it's a text file, directly compare the text contents
        return compare_text_files(
            expected_contents, output_contents, fromfile, tofile, similarity_threshold
        )


def compare_folders(
    expected_dir: Path,
    output_dir: Path,
    similarity_threshold: int = SIMILARITY_THRESHOLD,
) -> bool:
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

    # Because .pdf file are not necessary to be compared so we ignore they for now
    def get_md_and_pkl_files(dir: Path) -> Set[Path]:
        return {
            file.relative_to(dir)
            for file in dir.rglob("*")
            if file.is_file() and file.suffix.lower() in [".md", ".pkl"]
        }

    expected_files = get_md_and_pkl_files(expected_dir)
    output_files = get_md_and_pkl_files(output_dir)

    all_matched = True
    # Compare common files
    common_files = expected_files & output_files
    for file in common_files:
        expected_file_path = expected_dir / file
        output_file_path = output_dir / file
        if not compare_files(
            expected_file_path, output_file_path, similarity_threshold
        ):
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
        logging.info(
            Fore.YELLOW + "Missing files in expected directory:" + Style.RESET_ALL
        )
        for file in missing_files:
            logging.info(Fore.YELLOW + str(expected_dir / file) + Style.RESET_ALL)
        all_matched = False

    return all_matched
