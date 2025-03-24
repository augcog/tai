from pathlib import Path
from rag.file_conversion_router.classes.page import Page
import yaml
import os
def to_page(input_path):
    filepath = Path(input_path).parent
    filename = Path(input_path).stem
    filetype = Path(input_path).suffix.split(".")[1]
    with open(input_path, "r") as input_file:
        content = input_file.read()

    url = "URL_NOT_AVAILABLE"
    page = Page(filename, content={'text': content}, filetype=filetype, page_url=url)
    page.page_seperate_to_segments()
    page.tree_print()
    page.tree_segments_to_chunks()



to_page("/home/roar-tai-1/Desktop/yk/tai/output/61a-sp24-mt1/61a-sp24-mt1.md")
import pickle
from pathlib import Path
from rag.file_conversion_router.classes.chunk import Chunk

def load_pkl_file(file_path: Path):
    """Load and return the contents of a pickle file.

    Args:
        file_path (Path): Path to the pickle file.

    Returns:
        The contents of the pickle file.
    """
    with open(file_path, "rb") as file:
        data = pickle.load(file)
    return data

# Specify the path to the .pkl file
file_path = Path("/home/roar-tai-1/Desktop/yk/tai/output/61a-sp24-mt1/61a-sp24-mt1.pkl")

# Load the contents of the .pkl file
pkl_contents = load_pkl_file(file_path)


# Display the contents
print(pkl_contents[0].titles)
print(pkl_contents[0].content)
print(pkl_contents[0].chunk_url)
print(pkl_contents[0].page_num)