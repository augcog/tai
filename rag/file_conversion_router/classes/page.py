import string
from rag.file_conversion_router.classes.chunk import Chunk
import tiktoken
import pickle


class Page:

    PAGE_LENGTH_THRESHOLD = 500

    def __init__(self, pagename: str, content: dict, filetype: str, page_url: str = ""):
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

    def extract_headers_and_content(self, md_content):
        def count_consecutive_hashes(s):
            count = 0
            for char in s:
                if char == "#":
                    count += 1
                else:
                    break
            return count

        headers_content = []  # Stores tuples of ((header, level), content)
        curheader = None  # Current header, initially None
        current_content = ""  # Accumulates content for the current header
        in_code_block = False  # Indicates if inside a code block
        md_content = md_content.split('\n')
        for line in md_content:
            stripped_line = line.strip()
            if "```" in stripped_line:
                in_code_block = not in_code_block  # Toggle state

            if in_code_block:
                if curheader:
                    current_content += f"{line}\n"  # Add to content within code blocks
            else:
                if line.startswith('#'):
                    if curheader:
                        headers_content.append((curheader, current_content))  # Store previous header and its content
                    header = line
                    header_level = count_consecutive_hashes(header)
                    header = header.strip('#').strip()
                    curheader = (header, header_level)  # Start new header context
                    current_content = ""  # Reset content for new header
                else:
                    if curheader:  # Only accumulate content if within a header
                        current_content += f"{line}\n"

        # Append the last header and its content, if there was any header encountered
        if curheader:
            headers_content.append((curheader, current_content))

        return headers_content

    def page_seperate_to_segments(self) -> None:
        self.segments = [i for i in self.extract_headers_and_content(self.content['text'])]
        if not self.segments:
            # LEVEL 0 for no header found
            self.segments = [(("NO ANY HEADER DETECTED", 0),
                              self.content['text'])]

    def print_header_tree(self):
        result = ""
        for (title, level), _ in self.segments:
            indent = '--' * (level - 1)
            header_tag = f"(h{level})"
            result += f"{indent}{title} {header_tag}\n"
        return result

    def tree_print(self):
        new_filename = f"{self.pagename}_tree.txt"  # No need to use this
        top_header = []
        counter = 1

        for (header, level), content in self.segments:
            page_toc = ""
            page_path = ""
            segment = ""
            if len(top_header) < level:
                for i in range(len(top_header), level - 1):
                    top_header.append(("", [], i + 1))
                top_header.append((header, content, level))
            else:
                # Table of Contents
                page_toc += "(Table of Contents)\n"
                page_toc += f"{self.print_header_tree()}\n"

                # Page Path
                page_path += "(Page path)\n"
                first = True
                for h, c, l in top_header:
                    if first:
                        page_path += f"(h{l}) {h}"
                        first = not first
                    else:
                        page_path += " > "
                        page_path += f"(h{l}) {h}"
                # Segment Print
                segment += f"(Segment {counter})\n"
                header_list = [header[0] for header in top_header]
                for h, c, l in top_header:
                    hash_symbols = '#' * l
                    segment += f"{hash_symbols}{h} (h{l})\n"
                    segment += f"{c}\n"
                # Store the information in tree_segments
                self.tree_segments.append({'Page_table': page_toc, 'Page_path': header_list, 'Segment_print': segment})
                top_header = top_header[:(level - 1)]
                top_header.append((header, content, level))
                counter += 1

        # Handle the last segment
        all_headers = [header[0] for header in self.segments]
        if (header, level) == all_headers[-1]:
            page_toc = ""
            page_path = ""
            segment = ""
            # Table of Contents
            page_toc += "(Table of Contents)\n"
            page_toc += f"{self.print_header_tree()}\n"

            # Page Path
            page_path += "(Page path)\n"
            first = True
            for h, c, l in top_header:
                if first:
                    page_path += f"(h{l}) {h}"
                    first = not first
                else:
                    page_path += " > "
                    page_path += f"(h{l}) {h}"
            # Segment Print
            segment += f"(Segment {counter})\n"
            header_list = [header[0] for header in top_header]
            for h, c, l in top_header:
                hash_symbols = '#' * l
                segment += f"{hash_symbols}{h} (h{l})\n"
                segment += f"{c}\n"
            # Store the information in tree_segments
            self.tree_segments.append({'Page_table': page_toc, 'Page_path': header_list, 'Segment_print': segment})
            top_header = top_header[:(level - 1)]
            top_header.append((header, content, level))

    def tree_segments_to_chunks(self):
        def generate_hyperlink_header(header_text):
            """
            This function takes a header string, converts all characters to lowercase,
            and replaces all spaces with dashes to create a hyperlink-friendly header.

            Parameters:
            header_text (str): The header string to be converted.

            Returns:
            str: The converted hyperlink-friendly header string.
            """
            # Convert the string to lowercase
            lower_text = header_text.lower()

            # Replace spaces with dashes
            hyperlink_header = lower_text.replace(' ', '-')

            return hyperlink_header

        # seperate with recursive seperate
        for i in self.tree_segments:
            content_chunks = self.recursive_separate(i['Segment_print'], 400)
            for count, content_chunk in enumerate(content_chunks):
                headers = i['Page_path']
                urls = [f"{self.page_url}#{generate_hyperlink_header(header)}" for header in headers]
                page_path = ' > '.join(f"{item} (h{i + 1})" for i, item in enumerate(i['Page_path'])) + f" ({count})"
                self.chunks.append(Chunk(page_path, content_chunk, urls))
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
