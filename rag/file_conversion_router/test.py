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
file_path = Path("output_tmp/expected_output/debug/filename/filename.pkl")

# Load the contents of the .pkl file
pkl_contents = load_pkl_file(file_path)

# # Find out how many content chunks there are
num_chunks = len(pkl_contents)

# # Display the number of content chunks
# print(f"Number of content chunks: {num_chunks}")
# for idx, chunk in enumerate(pkl_contents):
#     print(f"Chunk {idx + 1}:")
#     print(chunk.titles)
#     print(chunk.content)
#     print(f"URL: {chunk.chunk_url}")
#     print(f"Page number: {chunk.page_num}")
#     print("-" * 40)  # Separator for better readability

# Specify the chunk number (1-based indexing for clarity)
chunk_number = 5  # Specify the chunk number you want

# Convert to zero-based index
chunk_index = chunk_number - 1

# Check if the index is valid
if 0 <= chunk_index < len(pkl_contents):
    selected_chunk = pkl_contents[chunk_index]
    print(f"Chunk {chunk_number}:")
    print(f"Titles: {selected_chunk.titles}")
    print(f"Content: {selected_chunk.content}")
    print(f"URL: {selected_chunk.chunk_url}")
    print(f"Page number: {selected_chunk.page_num}")
else:
    print(f"Invalid chunk number: {chunk_number}. Must be between 1 and {len(pkl_contents)}.")
