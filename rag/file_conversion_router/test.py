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
file_path = Path("output_tmp/expected_output/filename/filename.pkl")

# Load the contents of the .pkl file
pkl_contents = load_pkl_file(file_path)


# Display the contents
print(pkl_contents[0].titles)
print(pkl_contents[0].content)
print(pkl_contents[10].chunk_url)
print(pkl_contents[10].page_num)
print(pkl_contents[3].page_url)

