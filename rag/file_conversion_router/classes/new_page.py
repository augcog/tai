import string
import tiktoken
import pickle
import re




from file_conversion_router.classes.chunk import Chunk
class Page:
    PAGE_LENGTH_THRESHOLD = 20
    def __init__(self, filetype: str = "",page_name: str = "", page_url: str = "", index_helper: dict = None, content: dict = None,):
        """"""
        self.page_name = page_name
        self.content = content
        self.chunks = []
        self.page_url = page_url
        self.filetype = filetype
        self.index_helper = index_helper
        self.segments = []

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
        segments = []
        path_stack = []
        current_content = []
        def flush_segment():
            if path_stack:
                page_path = " > ".join(path_stack)
                segments.append({
                    "page_path": path_stack,
                    "index": self.index_helper[page_path],
                    "content": "\n".join(current_content).strip()
                })
                current_content.clear()
        for raw in md_content.splitlines():
            line = raw.rstrip("\n")
            stripped = line.strip()
            if stripped.startswith("#"):
                flush_segment()
                hashes, title = stripped.split(maxsplit=1)
                if not title:
                    print(f"Warning: Empty title found in line: {line}")
                    continue
                title = re.sub(r'\*+', '', title).strip()
                level = len(hashes)
                path_stack = path_stack[:level - 1]
                path_stack.append(title)
                continue
            current_content.append(line)
        flush_segment()
        return segments

    def page_separate_to_segments(self) -> None:
        self.segments = [i for i in self.extract_headers_and_content(self.content["text"])]

    def segments_to_chunks(self):
        for segment in self.segments:
            header = segment["page_path"]
            index = segment["index"]
            split_contents = self.recursive_separate(segment["content"], 400)
            for content_chunk in split_contents:
                self.chunks.append(
                    Chunk(
                        content=content_chunk,
                        titles=header,
                        chunk_url=self.page_url,
                        index = index,
                        is_split=(len(split_contents) > 1),
                    )
                )
        return self.chunks


    def to_chunk(self) -> None:
        self.page_separate_to_segments()
        self.chunks = self.segments_to_chunks()

    def chunks_to_pkl(self, output_path: str) -> None:
        with open(output_path, "wb") as f:
            pickle.dump(self.chunks, f)

    def token_size(self,sentence: str) -> int:
        encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        return len(encoding.encode(sentence))