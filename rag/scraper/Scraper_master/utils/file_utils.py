import os
import re
import shutil


def create_and_enter_dir(directory_name):
    """
    Creates a directory with the given name and enters it.

    Parameters:
    - directory_name (str): The name of the directory to be created and entered.
    """
    # print(directory_name)
    if directory_name:
        if not os.path.exists(directory_name):
            os.makedirs(directory_name, exist_ok=True)
        os.chdir(directory_name)


def delete_and_exit_dir():
    """
    Deletes the current directory and files in it and exits it.
    """
    cur_dir = os.getcwd()
    os.chdir("..")
    shutil.rmtree(cur_dir)


def cd_home(url):
    """
    Returns the home directory of a given URL.

    Parameters:
    - url (str): The URL whose home directory is needed.

    Returns:
    - str: The home directory URL.
    """
    return "/".join(url.split("/")[0:3])


def remove_consecutive_empty_lines(text):
    """
    Removes consecutive empty lines from a text, leaving only single empty lines.

    Parameters:
    - text (str): Text to be processed.

    Returns:
    - str: Cleaned text with single empty lines.
    """
    return re.sub(r"\n\s*\n", "\n\n", text)


def save_to_file(file_name, content):
    """
    Saves content to a file with the specified file name.

    Parameters:
    - file_name (str): The name of the file to save the content.
    - content (str): The content to be saved.
    """
    with open(file_name, "w", encoding="utf-8") as file:
        file.write(content)


# Scrape md
def replace_backslash_with_slash(path):
    """
    Replaces backslashes with slashes in a given file path.
    - path (str): The file path to modify.
    - Returns: The modified file path with slashes.
    """
    return path.replace("\\", "/")


def cd_back_link(url, num_parts_to_remove=1):
    """
    Navigates back in the URL structure by the specified number of parts.
    - url (str): The URL to navigate back from.
    - num_parts_to_remove (int, optional): The number of segments to remove from the end of the URL. Default is 1.
    - Returns: The modified URL.
    """
    if not url:
        return ""
    for _ in range(num_parts_to_remove):
        url = url.rsplit("/", 1)[0]
    return url
