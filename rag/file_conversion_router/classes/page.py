import string
from typing import Optional
from rag.file_conversion_router.classes.chunk import Chunk
import tiktoken
import pickle
import re
from pathlib import Path
import json


class Page:
    PAGE_LENGTH_THRESHOLD = 20

    def __init__(self, pagename: str, content: dict, filetype: str, page_url: str = "",
                 mapping_json_path: Optional[Path] = None):
        """
        Initialize a Page instance.

        Args:
            content (dict): A dictionary containing the page content.
            filetype (str): The file type (e.g., 'md', 'pdf').
            page_url (str): The page URL, default is an empty string.
            mapping_json_path (Optional[Path]): Path to the JSON mapping file, which stores
                the header-to-page mapping.
        """

        self.pagename = pagename
        self.content = content
        self.filetype = filetype
        self.page_url = page_url
        self.segments = []
        self.tree_segments = []
        self.chunks = []
        self.title_page_mapping = None  # 存储 JSON 映射数据
        self.mapping_pointer = 0  # 用于确保每次查找只往后搜索

        if filetype.lower() == "pdf":
            # 仅支持 PDF 时使用 JSON 映射文件
            if mapping_json_path:
                self.title_page_mapping = self.load_title_page_mapping(mapping_json_path)
                self.mapping_pointer = 0
        # 非 PDF 文件不需要页码，此时相关页码为 None

    def load_title_page_mapping(self, mapping_json_path: Path) -> list:
        """
        Load the JSON mapping file.

        Args:
            mapping_json_path (Path): Path to the JSON file containing the header-to-page mapping.
                Each entry in the file follows this structure:
                {
                    "type": "text",
                    "text": "1. (8.0 points) What Would Python Display?",
                    "text_level": 1,
                    "page_idx": 0
                }

        Returns:
            list: A list of JSON mapping data.
        """

        try:
            with open(mapping_json_path, 'r', encoding='utf-8') as f:
                mapping_data = json.load(f)
            print(f"Loaded title-page mapping: {mapping_data}")
            return mapping_data
        except Exception as e:
            print(f"Error loading title page mapping: {e}")
            return []

    def _get_page_num_for_title(self, header: str) -> Optional[int]:
        """
        Find the corresponding page number for a given header from the JSON mapping, searching only from the last matched position onward.

        Args:
            header (str): The header text parsed from the Markdown.

        Returns:
            Optional[int]: The corresponding page index if a match is found, otherwise None.
        """

        if not self.title_page_mapping:
            return None

        for i in range(self.mapping_pointer, len(self.title_page_mapping)):
            entry = self.title_page_mapping[i]
            # 比较时对两边字符串去空格处理
            if entry.get("text", "").strip() == header.strip():
                self.mapping_pointer = i + 1  # 更新指针，确保下次只向后搜索
                return entry.get("page_idx")
        return None

    def recursive_separate(self, response: str, token_limit: int = 400) -> list:
        """
        Recursively split the response into multiple paragraphs based on the token limit.

        Args:
            response (str): The text to be split.
            token_limit (int): Maximum number of tokens per paragraph.

        Returns:
            list: A list of split text segments.
        """

        def token_size(sentence: str) -> int:
            encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
            return len(encoding.encode(sentence))

        def rfind_punctuation(s: str, start: int, end: int) -> int:
            for i in range(end - 1, start - 1, -1):
                if s[i] in string.punctuation:
                    return i
            return -1

        msg_list = []
        tokens = token_size(response)
        if tokens > token_limit:
            start = 0
            while start < len(response):
                end = start
                while end < len(response) and token_size(response[start:end]) < token_limit:
                    end += 1

                if end < len(response):
                    split_pos = response.rfind('\n\n', start, end)
                    if split_pos == -1:
                        split_pos = response.rfind('\n', start, end)
                    if split_pos == -1:
                        split_pos = rfind_punctuation(response, start, end)
                    if split_pos == -1:
                        split_pos = response.rfind(' ', start, end)
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
        Extract headers and corresponding content from Markdown text while determining page numbers based on JSON mapping for PDF files.

        Args:
            md_content (str): Text content in Markdown format.

        Returns:
            list: A list of tuples where each element is ((header, page_num), content).
        """

        def count_consecutive_hashes(s: str) -> int:
            count = 0
            for char in s:
                if char == "#":
                    count += 1
                else:
                    break
            return count

        headers_content = []
        curheader = None
        current_content = ""
        in_code_block = False
        md_lines = md_content.split('\n')

        if self.filetype.lower() == 'pdf' and self.title_page_mapping is not None:
            for line in md_lines:
                stripped_line = line.strip()
                if "```" in stripped_line:
                    in_code_block = not in_code_block

                if in_code_block:
                    if curheader:
                        current_content += f"{line}\n"
                else:
                    if line.startswith('#'):
                        if curheader:
                            page_num_for_header = self._get_page_num_for_title(curheader[0])
                            headers_content.append(((curheader, page_num_for_header), current_content.strip()))
                        header_level = count_consecutive_hashes(line)
                        header = line.strip('#').strip()
                        curheader = (header, header_level)
                        current_content = ""
                    else:
                        current_content += f"{line}\n"
            if curheader:
                page_num_for_header = self._get_page_num_for_title(curheader[0]) + 1
                headers_content.append(((curheader, page_num_for_header), current_content.strip()))
        else:
            # 非 PDF 文件或无 JSON 映射时，每个标题对应的页码均为 None
            for line in md_lines:
                stripped_line = line.strip()
                if "```" in stripped_line:
                    in_code_block = not in_code_block

                if in_code_block:
                    if curheader:
                        current_content += f"{line}\n"
                else:
                    if line.startswith('#'):
                        if curheader:
                            headers_content.append(((curheader, None), current_content.strip()))
                        header_level = count_consecutive_hashes(line)
                        header = line.strip('#').strip()
                        curheader = (header, header_level)
                        current_content = ""
                    else:
                        current_content += f"{line}\n"
            if curheader:
                headers_content.append(((curheader, None), current_content.strip()))

        return headers_content

    def page_seperate_to_segments(self) -> None:
        self.segments = [i for i in self.extract_headers_and_content(self.content['text'])]
        if not self.segments:
            self.segments = [((("(NO ANY HEADER DETECTED)", 0), None), self.content['text'])]

    def print_header_tree(self) -> str:
        result = ""
        for (title, level), _ in self.segments:
            if level is not None:
                indent = '--' * (level - 1)
                result += f"{indent}{title}\n"
            else:
                result += f"{title} (hUnknown)\n"
        return result

    def tree_print(self):
        header_stack = []
        counter = 1
        self.tree_segments = []
        for (header, page_num), content in self.segments:
            level = header[1]
            header_title = header[0]
            while len(header_stack) >= level:
                header_stack.pop()
            header_stack.append(header_title)
            if not content.strip():
                continue
            segment_display = f"(Segment {counter})\n{'#' * level} {header_title} (h{level})\n{content.strip()}\n"
            page_toc = "(Table of Contents)\n" + self.print_header_tree() + "\n"
            tree_segment = {
                'Page_table': page_toc,
                'Page_path': header_stack.copy(),
                'Segment_print': segment_display,
            }
            if page_num is not None:
                tree_segment['page_num'] = page_num
            self.tree_segments.append(tree_segment)
            counter += 1

    def tree_segments_to_chunks(self):
        for segment in self.tree_segments:
            segment_title = segment['Page_path'][-1] if segment['Page_path'] else "(NO TITLE)"
            content_chunks = self.recursive_separate(segment['Segment_print'], 400)
            page_num = segment.get('page_num', None)
            for count, content_chunk in enumerate(content_chunks):
                if self.page_url != "":
                    if page_num is not None:
                        urls = f"{self.page_url}#page={page_num}"
                    else:
                        urls = self.page_url
                else:
                    urls = ""
                self.chunks.append(
                    Chunk(
                        content=content_chunk,
                        titles=segment_title,
                        chunk_url=urls,
                        page_num=page_num
                    )
                )
        return self.chunks

    def to_file(self, output_path: str) -> None:
        with open(output_path, "w", encoding='utf-8') as f:
            f.write(str(self))

    def to_chunk(self) -> None:
        self.page_seperate_to_segments()
        self.tree_print()
        self.chunks = self.tree_segments_to_chunks()

    def chunks_to_pkl(self, output_path: str) -> None:
        with open(output_path, "wb") as f:
            pickle.dump(self.chunks, f)
