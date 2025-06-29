import string
from typing import Optional
from rag.file_conversion_router.classes.chunk import Chunk
import tiktoken
import yaml
import pickle
import re
from pathlib import Path


class Page:
    PAGE_LENGTH_THRESHOLD = 20

    def __init__(
        self,
        pagename: str,
        content: dict,
        filetype: str,
        page_url: str = "",
        page_path: Optional[Path] = None,
    ):
        """
        Initialize a Page instance.

        Args:
            content (dict): Dictionary of page content attributes.
            filetype (str): Type of the file (e.g., 'md', 'pdf').
            page_url (str): URL of the page. Default is an empty string.
        """
        self.pagename = pagename
        self.content = content
        self.filetype = filetype
        self.page_url = page_url
        self.segments = []
        self.tree_segments = []
        self.chunks = []
        if filetype.lower() == "pdf" and page_path:
            self.page_numbers = self.load_metadata_page_numbers(page_path)
        else:
            self.page_numbers = None

    def load_metadata_page_numbers(self, page_path: Path):
        try:
            with open(page_path, "r", encoding="utf-8") as f:
                page_data = yaml.safe_load(f)
            loaded_page_numbers = [
                {
                    "page_num": page_info.get("page_num"),
                    "start_line": page_info.get("start_line"),
                }
                for page_info in page_data.get("pages", [])
            ]
            print(f"Loaded page numbers: {loaded_page_numbers}")
            return loaded_page_numbers
        except Exception as e:
            print(f"Error reading metadata: {e}")
            return []

    def recursive_separate(self, response: str, token_limit: int = 400) -> list:
        """
        Recursively separate a response into chunks based on token limit.

        Args:
            response (str): The text response to be separated.
            token_limit (int): Maximum number of tokens per chunk.

        Returns:
            list: List of separated text chunks.
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
                while (
                    end < len(response)
                    and token_size(response[start:end]) < token_limit
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

    def extract_headers_and_content(self, md_content):
        def count_consecutive_hashes(s):
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
        md_lines = md_content.split("\n")

        current_page_num = (
            self.page_numbers[0]["page_num"] if self.page_numbers else None
        )
        page_num_index = 1  # Start from the second page since we've assigned the first
        total_pages = len(self.page_numbers) if self.page_numbers else 0

        for line_num, line in enumerate(md_lines, start=1):
            stripped_line = line.strip()

            # Update current_page_num based on start_line
            if self.page_numbers:
                while (
                    page_num_index < total_pages
                    and line_num >= self.page_numbers[page_num_index]["start_line"]
                ):
                    current_page_num = self.page_numbers[page_num_index]["page_num"]
                    page_num_index += 1
            if "```" in stripped_line:
                in_code_block = not in_code_block

            if in_code_block:
                if curheader:
                    current_content += f"{line}\n"
            else:
                if line.startswith("#"):  # Identify headers
                    if curheader:  # Save the previous header and its content
                        headers_content.append(
                            ((curheader, current_page_num), current_content.strip())
                        )
                    header_level = count_consecutive_hashes(line)  # Count header level
                    header = line.strip("#").strip()
                    curheader = (header, header_level)  # Save the header and level
                    current_content = ""  # Reset content
                else:
                    current_content += f"{line}\n"

        # Append the last header and its content, if there was any header encountered
        if curheader:
            headers_content.append(
                ((curheader, current_page_num), current_content.strip())
            )

        return headers_content

    def page_seperate_to_segments(self) -> None:
        self.segments = [
            i for i in self.extract_headers_and_content(self.content["text"])
        ]
        if not self.segments:
            # LEVEL 0 for no header found
            self.segments = [("(NO ANY HEADER DETECTED)", 0), self.content["text"]]

    def print_header_tree(self) -> object:
        result = ""
        for (title, level), _ in self.segments:
            if level is not None:
                indent = "--" * (level - 1)
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
                "Page_table": page_toc,
                "Page_path": header_stack.copy(),
                "Segment_print": segment_display,
            }
            if page_num is not None:
                tree_segment["page_num"] = page_num
            self.tree_segments.append(tree_segment)
            counter += 1

    def tree_segments_to_chunks(self):
        for segment in self.tree_segments:
            segment_title = (
                segment["Page_path"][-1] if segment["Page_path"] else "(NO TITLE)"
            )
            content_chunks = self.recursive_separate(segment["Segment_print"], 400)
            page_num = segment.get("page_num", None)
            for count, content_chunk in enumerate(content_chunks):
                headers = segment["Page_path"]
                if self.page_url != "":
                    if page_num is not None:
                        urls = f"{self.page_url}#page={page_num}"
                    else:
                        urls = self.page_url
                else:
                    urls = ""
                # if self.page_url and page_num:
                #     urls = f"{self.page_url}#page={page_num}"
                # else:
                #     urls = "URL_NOT_AVAILABLE"
                page_path = (
                    " > ".join(
                        f"{item} (h{i + 1})"
                        for i, item in enumerate(segment["Page_path"])
                    )
                    + f" ({count})"
                )
                self.chunks.append(
                    Chunk(
                        content=content_chunk,
                        titles=segment_title,
                        chunk_url=urls,
                        # metadata={"page_path": page_path},  # Include page_path in metadata
                        page_num=page_num,
                    )
                )

        return self.chunks

    def to_file(self, output_path: str) -> None:
        """
        Write the page content to a file.

        Args:
            output_path (str): The path where the file will be written.
        """
        with open(output_path, "w") as f:
            f.write(str(self))

    def to_chunk(self) -> None:
        """
        Convert the page content to a list of Chunk objects.

        Returns:
            list[Chunk]: List of Chunk objects.
        """
        self.page_seperate_to_segments()
        self.tree_print()
        self.chunks = self.tree_segments_to_chunks()

    def chunks_to_pkl(self, output_path: str) -> None:
        """
        Write the page content chunks to a pkl file.

        Args:
            output_path (str): The path where the pkl file will be written.
        """
        with open(output_path, "wb") as f:
            pickle.dump(self.chunks, f)
