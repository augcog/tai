#!/usr/bin/env python3
import sys
import string
import tiktoken

def recursive_separate(response: str, token_limit: int = 400) -> list:
    """
    Recursively splits the text into multiple paragraphs and attempts to merge adjacent paragraphs,
    as long as the merged text remains within the token limit.

    If a paragraph's token count is below the token_limit, it will try to merge with the following paragraph
    (as long as the merged result remains within the token_limit). It also records and prints which original
    paragraphs were merged together.

    Args:
        response (str): The text to be split.
        token_limit (int): The maximum number of tokens allowed per paragraph.

    Returns:
        list: A list of text segments after splitting and merging.
    """

    def token_size(text: str) -> int:
        encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        return len(encoding.encode(text))

    def rfind_punctuation(s: str, start: int, end: int) -> int:
        for i in range(end - 1, start - 1, -1):
            if s[i] in string.punctuation:
                return i
        return -1

    def split_long_text(text: str, token_limit: int) -> list:
        """
        Splits a single text block if its token count exceeds token_limit using the original logic.
        """
        segments = []
        if token_size(text) <= token_limit:
            return [text.strip()]

        start = 0
        while start < len(text):
            end = start
            while end < len(text) and token_size(text[start:end]) < token_limit:
                end += 1
            if end < len(text):
                split_pos = text.rfind('\n\n', start, end)
                if split_pos == -1:
                    split_pos = text.rfind('\n', start, end)
                if split_pos == -1:
                    split_pos = rfind_punctuation(text, start, end)
                if split_pos == -1:
                    split_pos = text.rfind(' ', start, end)
                if split_pos == -1 or split_pos <= start:
                    split_pos = end - 1
                segments.append(text[start:split_pos].strip())
                start = split_pos + 1
            else:
                segments.append(text[start:end].strip())
                break
        return segments

    # First, split the original text by double newlines (assuming paragraphs are separated by two newlines)
    raw_paragraphs = [para.strip() for para in response.split("\n\n") if para.strip()]
    paragraphs = []
    # For each original paragraph, if its token count exceeds the limit, split it further
    for para in raw_paragraphs:
        if token_size(para) > token_limit:
            paragraphs.extend(split_long_text(para, token_limit))
        else:
            paragraphs.append(para)

    # Merge adjacent paragraphs and record the original paragraph indices
    merged = []
    merge_info = []  # Each element is a tuple (merged text, list of original paragraph indices)
    current = ""
    current_indices = []
    for idx, para in enumerate(paragraphs):
        if not current:
            current = para
            current_indices = [idx]
        else:
            candidate = current + "\n\n" + para
            if token_size(candidate) <= token_limit:
                current = candidate
                current_indices.append(idx)
            else:
                merged.append(current)
                merge_info.append((current, current_indices.copy()))
                current = para
                current_indices = [idx]
    if current:
        merged.append(current)
        merge_info.append((current, current_indices.copy()))

    # Print merge info
    print("Merge Info (each tuple: merged text, list of original paragraph indices):")
    for info in merge_info:
        print(info)

    return merged

if __name__ == "__main__":
    file_path = "/Users/yyk956614/tai/rag/output/md/01-Welcome_1pp/01-Welcome_1pp.md"
    token_limit = 400

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading file {file_path}: {e}")
        sys.exit(1)

    print(f"\nProcessing Markdown file: {file_path} with token limit: {token_limit}\n")
    merged_chunks = recursive_separate(content, token_limit=token_limit)

    print("\nFinal Merged Chunks:")
    for idx, chunk in enumerate(merged_chunks, start=1):
        print(f"Chunk {idx}:")
        print(chunk)
        print("-" * 40)
