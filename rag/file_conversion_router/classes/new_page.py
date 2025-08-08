import logging
import string
import tiktoken
import pickle
import re

from file_conversion_router.classes.chunk import Chunk
class Page:
    PAGE_LENGTH_THRESHOLD = 20
    def __init__(self, filetype: str = "",page_name: str = "", page_url: str = "", index_helper: dict = None, content: dict = None,file_path = None):
        """"""
        self.page_name = page_name
        self.content = content
        self.chunks = []
        self.page_url = page_url
        self.filetype = filetype
        self.index_helper = index_helper
        self.segments = []
        self.file_path = file_path

    def recursive_separate(self, response: str, token_limit: int = 400) -> list:
        """
        Recursively split the response into multiple paragraphs based on the token limit.

        Args:
            response (str): The text to be split.
            token_limit (int): Maximum number of tokens per paragraph.

        Returns:
            list: A list of split text segments.
        """

        def rfind_punctuation(s: str, start: int, end: int) -> int:
            for i in range(end - 1, start - 1, -1):
                if s[i] in string.punctuation:
                    return i
            return -1

        msg_list = []
        tokens = self.token_size(response)
        if not tokens:
            return msg_list
        if tokens > token_limit:
            start = 0
            while start < len(response):
                end = start
                while (
                    end < len(response)
                    and self.token_size(response[start:end]) < token_limit
                ):
                    end += 1

                if end < len(response):
                    split_pos = response.rfind("\n\n", start, end)
                    if split_pos == -1:
                        split_pos = response.rfind("\n", start, end)
                    if split_pos == -1:
                        split_pos = rfind_punctuation(response, start, end)
                    if split_pos == -1:
                        split_pos = response.rfind(" ", start, end)
                    if split_pos == -1 or split_pos <= start:
                        split_pos = end - 1
                    msg_list.append(response[start:split_pos].strip())
                    start = split_pos + 1
                else:
                    msg_list.append(response[start:end].strip())
                    break
        else:
            msg_list.append(response)

        return msg_list

    def extract_headers_and_content(self, md_content: str):
        """
        Split the markdown into content segments keyed by each header path.

        • self.index_helper : {path_tuple: (page_idx, line_no)}
          └─ line_no is 1-based, for the line *containing* the header.
        • Returns a list of dicts with the sliced content between this header
          and the next header (exclusive), stripped and chunked by
          self.recursive_separate().
        """
        segments = []
        lines = md_content.splitlines()

        # Order headers by their line number, NOT by page index
        # self.index_helper.items() is a dict_items view of (path_tuple, (page_idx, line_no))
        sorted_headers = sorted(
            self.index_helper.items(),
            key=lambda item: item[1][1]  # value[1] == line_number
        )

        for i, (path, (page_idx, line_no)) in enumerate(sorted_headers):
            start = line_no - 1  # convert to 0-based
            # The next header’s line OR EOF
            if i + 1 < len(sorted_headers):
                next_line_no = sorted_headers[i + 1][1][1]
                end = max(start + 1, next_line_no - 1)  # exclude next header
            else:
                end = len(lines)

            # Skip the header line itself; keep the body only
            body_lines = lines[start + 1:end]
            body = "\n".join(body_lines).strip()

            if body:  # ignore empty bodies
                for chunk in self.recursive_separate(body, 400):
                    segments.append({
                        "file_path": self.file_path,
                        "page_path": path,  # still the tuple
                        "index": page_idx,  # keep page index, not line no
                        "content": chunk
                    })

        return segments

    def page_separate_to_segments(self) -> None:
        self.segments = [i for i in self.extract_headers_and_content(self.content["text"])]

    def segments_to_chunks(self):
        for segment in self.segments:
            header = segment["page_path"]
            index = segment["index"]
            content = segment["content"]
            file_path = segment["file_path"]

            # Since content is already split in extract_headers_and_content,
            # we can directly create chunks
            self.chunks.append(
                Chunk(
                    content=content,
                    titles=header,
                    chunk_url=self.page_url,
                    index=index,
                    is_split=False,
                    file_path=file_path
                )
            )
        return self.chunks


    def to_chunk(self) -> None:
        self.page_separate_to_segments()
        self.chunks = self.segments_to_chunks()

    def chunks_to_pkl(self, output_path: str) -> None:
        with open(output_path, "wb") as f:
            pickle.dump(self.chunks, f)

    def token_size(self, sentence: str) -> int:
        encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        return len(encoding.encode(sentence))