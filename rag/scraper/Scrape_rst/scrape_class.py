import re
import pickle

class RSTParser:
    def __init__(self, filename):
        self.filename = filename
        self.header_levels = {}
        self.found_headers = {}
        self.redirected_from_content = []
        self.toctree_content = []
        
        # Auto-parse upon initialization
        self.parse_rst_headers_and_contents()

    def parse_rst_headers_and_contents(self):
        current_header = None
        skip_next_line = False
        not_header = False
        first_indent = False
        self.filename=self.filename.split("/")[-1]
        with open(self.filename, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            for i in range(len(lines)):
                line = lines[i]

                if line.startswith(".. ") and "::" not in line:
                    continue
                elif line.startswith(".. ") and "::" in line:
                    if line.startswith(".. redirect-from::"):
                        self.redirected_from_content.append(line)
                        cur = i + 1
                        while cur < len(lines) and lines[cur].strip() != "":
                            cur += 1

                        cur += 1
                        while cur < len(lines) and lines[cur].strip() != "":
                            self.redirected_from_content.append(lines[cur])
                            cur += 1
                    elif line.startswith(".. toctree::"):
                        # find the toctree content
                        cur = i + 1
                        while cur < len(lines) and lines[cur].strip() != "":
                            cur += 1

                        cur += 1
                        while cur < len(lines) and lines[cur].strip() != "":
                            match = re.search(r'<(.*?)>', lines[cur])
                            if match:
                                self.toctree_content.append(match.group(1).strip())
                            else:
                                self.toctree_content.append(lines[cur].strip())
                            cur += 1
                    not_header = True

                    
                

                line = line.strip()
                if not_header and line=="" and not first_indent:
                    first_indent = True
                elif not_header and line=="" and first_indent:
                    first_indent = False
                    not_header = False

                if skip_next_line:
                    skip_next_line = False
                    continue

                if i < len(lines) - 1 and re.match(r"[-=~^\"'#*_+@!$%&]{5,}", lines[i + 1].strip()) and not not_header:
                    underline_char = lines[i + 1].strip()[0]

                    if underline_char not in self.header_levels:
                        self.header_levels[underline_char] = len(self.header_levels) + 1

                    level = self.header_levels[underline_char]
                    level_name = f"h{level}"

                    current_header = (line, level_name)
                    # print(current_header)
                    self.found_headers[current_header] = []

                    skip_next_line = True

                elif i > 0 and re.match(r"[-=~^\"'#*_+@!$%&]{5,}", lines[i - 1].strip()) and i < (len(lines)-1) and re.match(r"[-=~^\"'#*_+@!$%&]{5,}", lines[i + 1].strip()) and not not_header:
                    underline_char = 2*lines[i + 1].strip()[0]
                    # print("!!!!!!!!!!!!!!" + underline_char)
                    if underline_char not in self.header_levels:
                        self.header_levels[underline_char] = len(self.header_levels) + 1

                    level = self.header_levels[underline_char]
                    level_name = f"h{level}" if level > 1 else "title"

                    current_header = (line, level_name)
                    # print(current_header)
                    self.found_headers[current_header] = []

                    skip_next_line = True

                elif current_header:
                    self.found_headers[current_header].append(line)
                    
    def has_redirected_from(self):
        return len(self.redirected_from_content) > 0

    def print_header_tree(self):
        stack = []
        result = ""  # Initialize an empty result string
        for (header, level), _ in self.found_headers.items():
            # Determine numerical level (e.g., 'title' becomes 0, 'h1' becomes 1, 'h2' becomes 2, etc.)
            num_level = 0 if level == 'title' else int(level[1:])

            # Pop from the stack until we find where this header belongs in the hierarchy
            while stack and stack[-1] >= num_level:
                stack.pop()

            # Append to the result string with indents based on its position in the stack
            indents = '--' * len(stack)
            result += f"{indents}{header} ({level})\n"  # Add to result string and add a newline character

            # Push this header's level onto the stack
            stack.append(num_level)

        return result  # Return the result string

    

    def save_parsed_data(self):
        new_filename = self.filename.replace('.rst', '_segment.txt')
        with open(new_filename, 'w', encoding='utf-8') as f:
            f.write("Parsed Headers, Levels and Contents:\n")
            for (header, level), content in self.found_headers.items():
                f.write(f"{level}: {header}\n")
                f.write("Contents:\n")
                for line in content:
                    f.write(f"  {line}\n")
                f.write("\n=====================================\n\n")

    def save_content_to_pkl(self, dict_list, filename='filename.pkl'):
        with open(filename, 'wb') as file:
            pickle.dump(dict_list, file)
    
    def concat_print(self):
        dict_list = []
        new_filename = self.filename.replace('.rst', '_tree.txt')
        top_header = []
        counter = 1
        heading_levels = {'h1': 1, 'h2': 2, 'h3': 3, 'h4': 4, 'h5': 5, 'h6': 6, 'h7':7}  # Add more if needed

        with open(new_filename, 'w', encoding='utf-8') as f:
            for (header, level), content in self.found_headers.items():
                page_toc = ""
                page_path = ""
                segment = ""
                if len(top_header) < heading_levels[level]:
                    for i in range(heading_levels[level] - len(top_header) - 1):
                        top_header.append(("", [], level))
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
                        hash_symbols = '#' * (heading_levels[l])
                        segment+=(f"{hash_symbols}{h} ({l})\n")
                        for line in c:  # iterate over each line in the content list
                            segment+=(f"{line}\n")
                        segment+=("\n")  # add an extra newline after the content
                        # Writing part
                    f.write(page_toc +"\n")
                    f.write(page_path+"\n\n")
                    f.write(segment  +"\n")
                    f.write('\n' + '-' * 80 + '\n')
                    dict_list.append({'Page_table': page_toc, 'Page_path': page_path, 'Segment_print': segment})
                    top_header = top_header[:(heading_levels[level]-1)]
                    top_header.append((header, content, level))

                    counter += 1
                # end of for loop
                if (header, level) == list(self.found_headers.keys())[-1]:
                    page_toc = ""
                    page_path = ""
                    segment = ""
                    if len(top_header) < heading_levels[level]:
                        for i in range(len(top_header), heading_levels[level]-1):
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
                            hash_symbols = '#' * (heading_levels[l])
                            segment+=(f"{hash_symbols}{h} ({l})\n")
                            for line in c:  # iterate over each line in the content list
                                segment+=(f"{line}\n")
                            segment+=("\n")  # add an extra newline after the content
                        # Writing part
                        f.write(page_toc +"\n")
                        f.write(page_path+"\n\n")
                        f.write(segment  +"\n")
                        f.write('\n' + '-' * 80 + '\n')
                        dict_list.append({'Page_table': page_toc, 'Page_path': page_path, 'Segment_print': segment})

                    
        self.save_content_to_pkl(dict_list, filename=f'{self.filename.split(".")[0]}.pkl')

# if __name__ == "__main__":
#     filename = 'index.rst'  # Replace with the name of your .rst file
#     parser = RSTParser(filename)
#     parser.print_header_tree()
#     parser.save_parsed_data()
#     parser.concat_print()
#     print("Toctree Content:")
#     print(parser.toctree_content)
