import os

def get_all_extension_under_dir(directory: str) -> set:
    """Get all unique file extensions under a specified directory.

    Args:
        directory (str): The path to the directory to search for files.

    Returns:
        set: A set of unique file extensions found in the directory.
    """
    extensions = set()
    for root, _, files in os.walk(directory):
        for file in files:
            ext = os.path.splitext(file)[1].lower()
            if ext:  # Ensure that the extension is not empty
                extensions.add(ext)
    return extensions

if __name__ == "__main__":
    # Example usage
    dir_path = "/home/bot/bot/yk/YK_final/courses/CS 61A/youtube"
    extensions = get_all_extension_under_dir(dir_path)
    print(f"Unique file extensions in '{dir_path}': {extensions}")