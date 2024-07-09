import re
import requests
import json
import pickle
from termcolor import colored


class MarkdownParser:
    def __init__(self, filename):
        self.filename = filename
        self.headers_content_list = []
        self.fail=False
        if self.fetch_data() == 1:
            self.fail=True

    @staticmethod
    def determine_level(s):
        level = 0
        if s.startswith('## Chapter'):
            level = 1
        else:
            for char in s:
                if char == "#":
                    level += 1
                else:
                    break
        return level

    @staticmethod
    def extract_headers_and_content(md_content):
        headers_content = []
        curheader = None
        curcontent = ""
        in_code_block = False

        for line in md_content:
            stripped_line = line.strip()

            # Check for code block start/end
            if "```" in stripped_line:
                in_code_block = not in_code_block

            # If inside a code block, continue without processing
            if in_code_block:
                if curheader:
                    curcontent += f"{line}"
                continue

            # Process headers outside code blocks
            if line.strip().startswith('#'):
                if curheader:
                    headers_content.append((curheader, curcontent))

                header = line
                header_level = MarkdownParser.determine_level(header)
                header = header.strip('#').strip()
                curheader = (header, header_level)
                curcontent = ""
            else:
                if curheader:
                    curcontent += f"{line}"

        # Add the last header and content if exists
        if curheader:
            headers_content.append((curheader, curcontent))

        return headers_content


    def fetch_data(self):
        try:
            with open(self.filename, 'r', encoding='utf-8') as file:
                md_content = file.readlines()
            self.headers_content_list = MarkdownParser.extract_headers_and_content(md_content)
        except FileNotFoundError:
            print(colored(f"File '{self.filename}' not found.", "red"))
            return 1
        self.headers_content_list = MarkdownParser.extract_headers_and_content(md_content)

    def print_header_tree(self):
        result = ""
        all_headers = [header[0] for header in self.headers_content_list]

        for title, level in all_headers:
            indent = '--' * (level - 1)
            header_tag = f"(h{level})"
            result+=f"{indent}{title} {header_tag}\n"
        return result

    def print_segment(self):
        new_filename = "textbook/segment.txt"
        with open(new_filename, 'w', encoding='utf-8') as f:
            f.write("Parsed Headers, Levels and Contents:\n")
            for (header, level), content in self.headers_content_list:
                f.write(f"h{level}: {header}\n")
                f.write("Contents:\n")
                f.write(content)
                f.write("\n=====================================\n\n")

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
                if len(top_header) < level:
                    for i in range(len(top_header), level-1):
                        top_header.append(("", [], i+1))
                    top_header.append((header, content, level))
                else:                    
                    # Table of Contents
                    page_toc+=("(Table of Contents)\n")
                    page_toc+=(f"{self.print_header_tree()}\n")
                    
                    # Page Path
                    page_path+=("(Page path)\n")
                    first = True
                    for h, c, l in top_header:
                        if first:
                            page_path+=(f"(h{l}) {h}")
                            first = not first
                        else:
                            page_path+=(" > ")
                            page_path+=(f"(h{l}) {h}")
                    # Segment Print
                    segment+=(f"(Segment {counter})\n")
                    for h, c, l in top_header:
                        hash_symbols = '#' * l
                        segment+=(f"{hash_symbols}{h} (h{l})\n")
                        segment+=(f"{c}\n")
                        # Writing Part
                    f.write(page_toc +"\n")
                    f.write(page_path+"\n\n")
                    f.write(segment  +"\n")
                    f.write('\n' + '-' * 80 + '\n')
                    top_header = top_header[:(level-1)]
                    top_header.append((header, content, level))
                    dict_list.append({'Page_table': page_toc, 'Page_path': page_path, 'Segment_print': segment})
                    counter += 1
                # end of for loop
                all_headers = [header[0] for header in self.headers_content_list]
                if (header, level) == all_headers[-1]:
                    page_toc = ""
                    page_path = ""
                    segment = ""
                    # Table of Contents
                    page_toc+=("(Table of Contents)\n")
                    page_toc+=(f"{self.print_header_tree()}\n")
                    
                    # Page Path
                    page_path+=("(Page path)\n")
                    first = True
                    for h, c, l in top_header:
                        if first:
                            page_path+=(f"(h{l}) {h}")
                            first = not first
                        else:
                            page_path+=(" > ")
                            page_path+=(f"(h{l}) {h}")
                    # Segment Print
                    segment+=(f"(Segment {counter})\n")
                    for h, c, l in top_header:
                        hash_symbols = '#' * l
                        segment+=(f"{hash_symbols}{h} (h{l})\n")
                        segment+=(f"{c}\n")
                        # Writing Part
                    f.write(page_toc +"\n")
                    f.write(page_path+"\n\n")
                    f.write(segment  +"\n")
                    f.write('\n' + '-' * 80 + '\n')
                    top_header = top_header[:(level-1)]
                    top_header.append((header, content, level))
                    dict_list.append({'Page_table': page_toc, 'Page_path': page_path, 'Segment_print': segment})
        self.save_content_to_pkl(dict_list, filename=f'{self.filename}.pkl')


# TODO
parser = MarkdownParser('textbook/MLS.mmd')

print(parser.print_header_tree())
parser.print_segment()
parser.print_segment()
parser.concat_print()