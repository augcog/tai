import requests
from bs4 import BeautifulSoup, Tag, NavigableString

class ContentExtractor:
    
    def __init__(self, url):
        self.url = url
        self.headings = ['title', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6']
        self.heading_levels = {heading: index+1 for index, heading in enumerate(self.headings)}
        self.content_dict = {}
        self._fetch_content()

    @staticmethod
    def _clean_text(text):
        # Remove non-ASCII characters
        cleaned = ''.join(char for char in text if 32 <= ord(char) <= 126 or char in '\n\t')
        return cleaned
    
    @staticmethod
    def _has_headings(tag):
        # Base condition: if the tag itself is a heading
        if tag.name in ['h1', 'h2', 'h3', 'h4', 'h5', 'h6']:
            return True
        # Recursively check all children of the tag
        for child in tag.children:
            if isinstance(child, Tag) and ContentExtractor._has_headings(child):
                return True
        return False

    def _gather_content(self, tag):
        content = []
        print(tag.text)
        if tag.text=="ROS Tutorials":
            print(tag.find_next_siblings()[2])
        for sibling in tag.find_next_siblings():
            # Check if the sibling itself or any of its descendants is a heading
            
            if sibling.name in self.headings or self._has_headings(sibling):
                
                break
            content.append(sibling)
            # print(sibling.tag)
        return content

    def print_text_with_indentation(self, element, indent=0):
        result_str = ''

        if isinstance(element, Tag):
            # If the tag is <pre>, just return its text
            if element.name == "pre":
                return '<code>\n' + '\n'.join(['\t' + line for line in element.get_text().splitlines()]) + '\n' + '</code>\n'

            if element.name == "code":
                return '<code>' + element.get_text() + '</code>'

            # Iterate over children to generate result
            for child in element.children:
                child_str = self.print_text_with_indentation(child, indent+1 if element.name == "li" else indent)
                result_str += child_str

                # Check if the next sibling is a span or if the current child is a NavigableString. 
                # If so, don't append a newline.
                # print(child.name)

                # print(child)
                if(child.name in ["span", "em", "code", "strong"]) or (not isinstance(child, NavigableString) and child.has_attr('href')):
                    # print(child)
                    result_str += ''
                elif ((child.next_sibling and isinstance(child.next_sibling, Tag) and child.next_sibling.name in ["span", "em","code", "strong"])) or (child.next_sibling and not isinstance(child.next_sibling, NavigableString) and child.next_sibling.has_attr('href')):                    
                    result_str += ''                
                elif result_str and result_str[-1] != '\n' and not isinstance(child, NavigableString):
                    result_str += '\n'


        elif isinstance(element, NavigableString) and element.strip():  # Avoid empty strings
            # print(element)
            result_str += ' ' * indent * 2 + element

        return result_str


    def _fetch_content(self):
        response = requests.get(self.url)
        response.raise_for_status()  
        soup = BeautifulSoup(response.text, 'html.parser')
        
        for tag in soup.find_all(self.headings):
            header_text = f"{tag.name}: {self._clean_text(tag.getText())}"
            if header_text not in self.content_dict:
                segment_content = self._gather_content(tag)
                segment_soup = BeautifulSoup(''.join(str(t) for t in segment_content), 'html.parser')
                # self.content_dict[header_text] = self._clean_text(segment_soup.getText())
                self.content_dict[header_text] = self._clean_text(self.print_text_with_indentation(segment_soup))
                # self.content_dict[header_text] = self._clean_text(segment_soup.prettify())
                # self.content_dict[header_text] = self.print_text_with_indentation(segment_soup)


    def save_to_file(self, filename='good.txt'):
        with open(filename, 'w', encoding='utf-8') as file:
            
            for idx, (header, content) in enumerate(self.content_dict.items(), 1):
                tag, _ = header.split(": ", 1)
                hash = '#' * (self.heading_levels[tag]-1)
                file.write(f"Segment {idx}\n")
                file.write(f"{hash}{header}: \n")
                file.write(content)
                file.write('\n' + '-'*40 + '\n')

    def print_tree_structure(self):
        for header in self.content_dict.keys():
            tag, content = header.split(": ", 1)
            
            # Check if tag is "Title", treat it as root
            if tag == 'title':
                level = 0
            else:
                level = int(tag[1])
                
            indent = '  ' * level  # Two spaces for each indentation level
            print(f"{indent}{tag}: {content}")

    def concat_print(self, filename='tree.txt'):
        top_header = []
        counter = 1

        # Open the file in write mode
        with open(filename, 'w') as f:
            for (header, content) in self.content_dict.items():
                tag, _ = header.split(": ", 1)
                if len(top_header) < self.heading_levels[tag]:
                    top_header.append((header, content))
                else:
                    # Write to the file instead of printing
                    # print(top_header)
                    f.write(f"Segment {counter}\n")
                    for h, c in top_header:
                        # print(h)
                        tag, _ = h.split(": ", 1)
                        hash = '#' * (self.heading_levels[tag]-1)
                        f.write(f"{hash}{h}: \n")
                        f.write(c+"\n\n")
                    f.write('\n' + '-'*40 + '\n')
                    top_header = top_header[:self.heading_levels[tag]-1]
                    top_header.append((header, content))
                    if(header, content) == list(self.content_dict.items())[-1]:
                        f.write(f"Segment {counter}\n")
                        for h, c in top_header:
                            # print(h)
                            tag, _ = h.split(": ", 1)
                            hash = '#' * (self.heading_levels[tag]-1)
                            f.write(f"{hash}{h}: \n")
                            f.write(c+"\n\n")
                        f.write('\n' + '-'*40 + '\n')
                    counter += 1
                    
                
                


            



# Usage:
#ROS2
# url = "https://docs.ros.org/en/galactic/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html"
# url = "https://docs.ros.org/en/galactic/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html"
# Numpy
# url = "https://numpy.org/install/" #Skpped some parts
# Python
# url = 'https://docs.python.org/3/tutorial/controlflows.html#positional-or-keyword-arguments' # pretty good
# url = 'https://docs.python.org/3/tutorial/datastructures.html#using-lists-as-queues' # pretty good
# pytorch
url = 'http://wiki.ros.org/ROS/Tutorials?action=print'
# url = "https://docs.ros.org/en/foxy/index.html"
extractor = ContentExtractor(url)
extractor.save_to_file('good.txt')
extractor.print_tree_structure()
extractor.concat_print('tree.txt')