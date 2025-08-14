from file_conversion_router.classes.chunk import Chunk
import pickle

def read_chunk(pkl_file_path):
    """
    Read a chunk from a pickle file and return it as a Chunk object.

    Args:
        pkl_file_path (str): Path to the pickle file containing the chunk.

    Returns:
        Chunk: The chunk read from the file.
    """
    with open(pkl_file_path, "rb") as f:
        chunks = pickle.load(f)
    for i in range(len(chunks)):
        chunk = chunks[i]
        print(f"\n--- Chunk {i}---")
        print(f"Chunk content: {chunk.content}")  # FULL content, not repr
        print(f"Chunk titles: {chunk.titles}")
        print(f"Chunk URL: {chunk.chunk_url}")
        print(f"Chunk is_split: {chunk.is_split}")
        print(f"Chunk index: {chunk.index}")
        print(f"Chunk file_path: {chunk.file_path}")

if __name__ == "__main__":
    pkl_file_path = "/home/bot/bot/yk/YK_final/courses_out/CS 61A_output/staff/Course Staff CS 61A Summer 2025/Course Staff CS 61A Summer 2025.html.pkl"
    chunk = read_chunk(pkl_file_path)
