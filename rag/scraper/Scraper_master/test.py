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
file_path = Path("/home/bot/roarai/rag/scraper/Scraper_master/opencv_pkl/tutorial_py_table_of_contents_bindings/tutorial_py_bindings_basics/tutorial_py_bindings_basics.pkl")

# Load the contents of the .pkl file
pkl_contents = load_pkl_file(file_path)


# Display the contents
print(pkl_contents[0].titles)
print(pkl_contents[0].content)
print(pkl_contents[0].chunk_url)
