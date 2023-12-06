import re

def parse_rst_headers_and_contents(filename):
    header_levels = {}
    found_headers = {}
    current_header = None
    skip_next_line = False
    not_header = False
    first_indent = False
    redirected_from_content = []
    toctree_content = []
    
    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            line = lines[i]

            if line.startswith(".. ") and "::" not in line:
                continue
            elif line.startswith(".. ") and "::" in line:
                if line.startswith(".. redirect-from::"):
                    redirected_from_content.append(line)
                    cur = i+1
                    while(lines[cur].strip()!=""):
                        cur+=1
                    cur+=1

                    while(lines[cur].strip()!=""):
                        redirected_from_content.append(lines[cur])
                        cur += 1
                elif line.startswith(".. toctree::"):
                    # find the toctree content
                    cur = i+1
                    while(lines[cur].strip()!=""):
                        cur+=1
                    cur+=1
                    while(lines[cur].strip()!=""):
                        toctree_content.append(lines[cur].strip())
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

            if i < len(lines) - 1 and re.match(r"[-=~^\"'#]+$", lines[i + 1].strip()) and not not_header:
                underline_char = lines[i + 1].strip()[0]

                if underline_char not in header_levels:
                    header_levels[underline_char] = len(header_levels) + 1

                level = header_levels[underline_char]
                level_name = f"h{level}"

                current_header = (line, level_name)
                found_headers[current_header] = []

                skip_next_line = True

            elif i > 0 and re.match(r"[-=~^\"'#]+$", lines[i - 1].strip()) and re.match(r"[-=~^\"'#]+$", lines[i + 1].strip()) and not not_header:
                underline_char = lines[i + 1].strip()[0]

                if underline_char not in header_levels:
                    header_levels[underline_char] = len(header_levels) + 1

                level = header_levels[underline_char]
                level_name = f"h{level}" if level > 1 else "title"

                current_header = (line, level_name)
                found_headers[current_header] = []

                skip_next_line = True

            elif current_header:
                found_headers[current_header].append(line)

    return found_headers, redirected_from_content, toctree_content

def print_header_tree(parsed_data):
    stack = []
    for (header, level), _ in parsed_data.items():
        # Determine numerical level (e.g., 'title' becomes 0, 'h1' becomes 1, 'h2' becomes 2, etc.)
        num_level = 0 if level == 'title' else int(level[1:])

        # Pop from the stack until we find where this header belongs in the hierarchy
        while stack and stack[-1] >= num_level:
            stack.pop()

        # Print the header with indents based on its position in the stack
        indents = '--' * len(stack)
        print(f"{indents}{header} ({level})")

        # Push this header's level onto the stack
        stack.append(num_level)





filename = 'index.rst'  # Replace with the name of your .rst file
parsed_data, source, toctree_content= parse_rst_headers_and_contents(filename)
print("Toctree Content:")
print(toctree_content)

# Generate new filename for the .txt file
new_filename = filename.replace('.rst', '_segment.txt')

# Write parsed data to the new .txt file
with open(new_filename, 'w', encoding='utf-8') as f:
    f.write("Parsed Headers, Levels and Contents:\n")
    for (header, level), content in parsed_data.items():
        f.write(f"{level}: {header}\n")
        f.write("Contents:\n")
        for line in content:
            f.write(f"  {line}\n")
        f.write("\n=====================================\n\n")

# Call the new function to print the header tree
print("Header Tree:")
print_header_tree(parsed_data)

def concat_print(source, parsed_data, filename='tree.rst'):
    new_filename = filename.replace('.rst', '_tree.txt')
    top_header = []
    counter = 1
    heading_levels = {'h1': 1, 'h2': 2, 'h3': 3, 'h4': 4}  # Add more if needed

    with open(new_filename, 'w', encoding='utf-8') as f:
        for i in source:
            f.write(i)
        for (header, level), content in parsed_data.items():

            if len(top_header) < heading_levels[level]:
                for i in range(heading_levels[level], len(top_header)):
                    top_header.append(("", [], level))
                top_header.append((header, content, level))
            else:
                f.write(f"Segment {counter}\n")
                for h, c, l in top_header:
                    hash_symbols = '#' * (heading_levels[l] - 1)
                    f.write(f"{hash_symbols}{h}\n")
                    for line in c:  # iterate over each line in the content list
                        f.write(f"{line}\n")
                    f.write("\n")  # add an extra newline after the content
                f.write('\n' + '-' * 40 + '\n')
                top_header = top_header[:(heading_levels[level]-1)]
                top_header.append((header, content, level))

                if (header, level) == list(parsed_data.keys())[-1]:
                    f.write(f"Segment {counter}\n")
                    for h, c, l in top_header:
                        hash_symbols = '#' * (heading_levels[l] + 1)
                        f.write(f"{hash_symbols} {h}\n")
                        for line in c:  # iterate over each line in the content list
                            f.write(f"{line}\n")
                        f.write("\n")  # add an extra newline after the content
                    f.write('\n' + '-' * 40 + '\n')
                counter += 1




# print(parsed_data)
# for i,l in parsed_data.items():
    # print(i)
    # for kk in l: 
    #     print(kk)

concat_print(source, parsed_data, filename=filename)






