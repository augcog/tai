import re
import requests
import json
from termcolor import colored
import pickle

class MarkdownParser:
    
    def __init__(self, url, filename):
        self.url = url
        self.filename = filename
        self.headers_content_dict = None
        self.fail=False
        if self.fetch_data() == 1:
            self.fail=True

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
        headers_content = {}
        curheader = ()
        in_code_block = False  # New variable to track if we're inside a code block

        for line in md_content:
            stripped_line = line.strip()

            # Check for code block start/end
            if "```" in stripped_line:
                in_code_block = not in_code_block  # Toggle in_code_block state

            # If inside a code block, continue without processing
            if in_code_block:
                if curheader:
                    headers_content[curheader] += f"{line}"
                continue

            # Process headers outside code blocks
            if line.startswith('#'):
                header = line
                header_level = MarkdownParser.count_consecutive_hashes(header)
                header = header.strip('#').strip()
                headers_content[(header, header_level)] = ""
                curheader = (header, header_level)
            else:
                if not curheader:
                    continue
                headers_content[curheader] += f"{line}"

        return headers_content


    def fetch_data(self):
        headers = {'Accept': 'application/json'}
        response = requests.get(self.url, headers=headers)
        data = response.json()
        md_content = data['payload']['blob']['rawLines']
        self.headers_content_dict = MarkdownParser.extract_headers_and_content(md_content)

    def print_header_tree(self):
        result = ""
        for title, level in self.headers_content_dict.keys():
            indent = '--' * (level - 1)
            header_tag = f"(h{level})"
            result+=f"{indent}{title} {header_tag}\n"
        return result

    def print_segment(self):
        new_filename = f"{self.filename}_segment.txt"
        with open(new_filename, 'w', encoding='utf-8') as f:
            f.write("Parsed Headers, Levels and Contents:\n")
            for (header, level), content in self.headers_content_dict.items():
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
            for (header, level), content in self.headers_content_dict.items():
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
                            page_path+=(f"({l}) {h}")
                            first = not first
                        else:
                            page_path+=(" > ")
                            page_path+=(f"({l}) {h}")
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
                if (header, level) == list(self.headers_content_dict.keys())[-1]:
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
                            page_path+=(f"({l}) {h}")
                            first = not first
                        else:
                            page_path+=(" > ")
                            page_path+=(f"({l}) {h}")
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

# # Example usage:
# url = "https://github.com/carla-simulator/carla/blob/master/Docs/index.md?plain=1"
# parser = MarkdownParser(url, )
# parser.print_header_tree()
# parser.print_segment()
# parser.concat_print()
