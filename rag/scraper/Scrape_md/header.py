import re
import requests
import json
from termcolor import colored
import pickle


class MarkdownParser:

    def __init__(self, url, filename):
        self.url = url
        self.filename = filename
        self.headers_content_list = None  # Changed from dict to list
        self.fail = False
        if self.fetch_data() == 1:
            self.fail = True

    @staticmethod
    def count_consecutive_hashes(s):
        count = 0
        for char in s:
            if char == "#":
                count += 1
            else:
                break
        return count

    @staticmethod
    def extract_headers_and_content(md_content):
        headers_content = []  # Stores tuples of ((header, level), content)
        curheader = None  # Current header, initially None
        current_content = ""  # Accumulates content for the current header
        in_code_block = False  # Indicates if inside a code block

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
                    header_level = MarkdownParser.count_consecutive_hashes(header)
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

    def fetch_data(self):
        headers = {'Accept': 'application/json'}
        response = requests.get(self.url, headers=headers)
        data = response.json()
        md_content = data['payload']['blob']['rawLines']
        self.headers_content_list = MarkdownParser.extract_headers_and_content(md_content)  # Adjusted to list

    def print_header_tree(self):
        result = ""
        for (title, level), _ in self.headers_content_list:
            indent = '--' * (level - 1)
            header_tag = f"(h{level})"
            result += f"{indent}{title} {header_tag}\n"
        return result

    def print_segment(self):
        new_filename = f"{self.filename}_segment.txt"
        with open(new_filename, 'w', encoding='utf-8') as f:
            f.write("Parsed Headers, Levels and Contents:\n")
            for (header, level), content in self.headers_content_list:
                f.write(f"h{level}: {header}\n")
                f.write("Contents:\n")
                f.write(content)
                f.write("\n=====================================\n\n")

    def save_content_to_pkl(self, list_data, filename='filename.pkl'):
        with open(filename, 'wb') as file:
            pickle.dump(list_data, file)

    def save_content_to_pkl(self, dict_list, filename='filename.pkl'):
        with open(filename, 'wb') as file:
            pickle.dump(dict_list, file)

    def concat_print(self):
        dict_list = []
        new_filename = f"{self.filename}_tree.txt"
        top_header = []
        counter = 1
        with open(new_filename, 'w', encoding='utf-8') as f:
            for (header, level), content in self.headers_content_list:
                page_toc = ""
                page_path = ""
                segment = ""
                # Update the top_header list to the correct hierarchy level
                if len(top_header) < level:
                    for i in range(len(top_header), level):
                        top_header.append(("", "", i + 1))
                    top_header.append((header, content, level))
                else:
                    top_header = top_header[:level - 1]
                    top_header.append((header, content, level))

                # Table of Contents
                page_toc += "(Table of Contents)\n"
                page_toc += f"{self.print_header_tree()}\n"

                # Page Path
                page_path += "(Page path)\n"
                first = True
                for h, c, l in top_header:
                    if first:
                        page_path += f"({l}) {h}"
                        first = not first
                    else:
                        page_path += " > "
                        page_path += f"({l}) {h}"

                # Segment Print
                segment += f"(Segment {counter})\n"
                for h, c, l in top_header:
                    hash_symbols = '#' * l
                    segment += f"{hash_symbols}{h} (h{l})\n"
                    segment += f"{c}\n"

                # Writing Part
                f.write(page_toc + "\n")
                f.write(page_path + "\n\n")
                f.write(segment + "\n")
                f.write('\n' + '-' * 80 + '\n')

                dict_list.append({'Page_table': page_toc, 'Page_path': page_path, 'Segment_print': segment})
                counter += 1

        self.save_content_to_pkl(dict_list, filename=f'{self.filename}.pkl')

    # Continue with the example usage and instantiation as previously described

# Example usage:
# url = "https://github.com/carla-simulator/carla/blob/master/Docs/adv_opendrive.md?plain=1"
# parser = MarkdownParser(url, "TEST")
# parser.print_header_tree()
# parser.print_segment()
# parser.concat_print()
