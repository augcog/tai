import requests
from bs4 import BeautifulSoup
import os
from header import MarkdownParser
import urllib.robotparser as robotparser
import time
import re
from termcolor import colored
from urllib.parse import urljoin
from markdownify import markdownify as md

def remove_slash_and_hash(link):
    """
    Removes trailing slash (if present) and hash fragment from a given URL.

    Parameters:
    - link (str): The URL from which the last segment should be modified.

    Returns:
    - str: The modified URL without the trailing slash and hash fragment.
    """
    # Remove hash fragment
    if not link:
        return link
    link = link.split('#')[0]
    if not link:
        return link
    # print(link)
    # Remove trailing slash
    if link[-1] == '/':
        link = link[:-1]

    return link



def create_and_enter_dir(directory_name):
    # Create the directory if it doesn't exist
    if not os.path.exists(directory_name):
        os.mkdir(directory_name)
    
    # Change the current working directory
    os.chdir(directory_name)

def cd_home(url):
    return '/'.join(url.split('/')[0:3])

def get_crawl_delay(site_url, user_agent="*"):
    robots_url = site_url.rstrip('/') + '/robots.txt'
    
    rp = robotparser.RobotFileParser()
    rp.set_url(robots_url)
    try:
        rp.read()
        delay = rp.crawl_delay(user_agent)
        if delay:
            return delay
        else:
            return 0
    except:
        print("Error accessing or parsing robots.txt.")
        return 0

def process_links_and_save(links, dir_name, delay, content_tags):
    """
    Processes a list of links by converting them to markdown, cleaning up the markdown content, and saving the results to files. 
    The files are saved within directories named after the last segment of each link.
    
    Parameters:
    - links (list): A list of URLs to be processed.
    - dir_name (str): The name of the main directory where the results should be saved.
    - delay (int/float): The delay in seconds to wait between processing each link.

    Returns:
    None
    """
    create_and_enter_dir(dir_name)
    for link in links:
        if link[-1] == '/': 
            link = link[:-1]
        filename = link.split('/')[-1]
        filename = filename.split('.')[0]
        print(link)
        print(filename)
        
        markdown_result = html_to_markdown(link, content_tags)
        if markdown_result == 1:
            continue
        
        cleaned_markdown = remove_consecutive_empty_lines(markdown_result)
        
        cur_dir = os.getcwd()
        create_and_enter_dir(filename)
        
        save_to_file(f'{filename}.md', cleaned_markdown)

        parser = MarkdownParser(f'{filename}')
        parser.print_header_tree()
        parser.print_segment()
        parser.concat_print()
        
        os.chdir(cur_dir)
        time.sleep(delay)

def extract_unique_links(url, root, root_regex, root_filename, content_tags, delay=0, found_links=[]):
    """
    Extract and print unique links from a given URL that start with a specified root.
    
    Parameters:
    - url (str): The URL from which links are to be extracted.
    - root (str): The root URL which extracted links should start with to be considered.
    
    Returns:
    - list: A list of unique links that match the criteria.
    """
    print(colored(f"found_links{found_links}", 'red'))
    reqs = requests.get(url)
    soup = BeautifulSoup(reqs.text, 'html.parser')
    
    unique_links = set()  # Create an empty set to store unique links
    
    for link in soup.find_all('a'):
        href = link.get('href')
        href = remove_slash_and_hash(href)
        if href in found_links or not href:
            continue
        # print(href)
        clean_href = ''
        if href.startswith('http://') or href.startswith('https://'):
            clean_href = remove_slash_and_hash(href)
        elif href and href.startswith('#'):
            continue
        elif href and href.startswith('/'):
            clean_href = (remove_slash_and_hash(urljoin(cd_home(root), href)))
        elif href and not (href.startswith('http://') or href.startswith('https://')) and '/' in href:
            clean_href = (remove_slash_and_hash(urljoin(root, href)))
        print(clean_href)
        if re.match(root_regex, clean_href) and clean_href not in found_links:
            unique_links.add(clean_href)

    links = list(unique_links)

    print(colored(f"links{links}", 'yellow'))
    if not links:
        return
    process_links_and_save(links, root_filename, delay, content_tags=content_tags)
    found_links.extend(links)
    cur_dir = os.getcwd()
    for link in links:
        remove_slash_and_hash(link)
        filename=link.split('/')[-1]
        filename=filename.split('.')[0]
        print(colored(f"filename{filename}", 'green'))
        print(colored(f"link{link}", 'green'))
        extract_unique_links(link, root, root_regex, filename, content_tags,delay, found_links)
        os.chdir(cur_dir)

def html_to_markdown(url, content_tags):
    # Fetch HTML content from the URL
    try:
        response = requests.get(url)
        response.raise_for_status()  # Raise an exception for HTTP errors
    except requests.RequestException as e:
        print(f"Failed to retrieve the URL due to: {e}")
        return 1

    # Parse the HTML content
    soup = BeautifulSoup(response.text, 'html.parser')

    markdown_outputs = []

    for tag_type, tag_attr in content_tags:
        # Find the specific tags
        content = soup.find_all(tag_type, tag_attr)
        for item in content:
            # Convert each HTML item to Markdown
            markdown = md(str(item), heading_style="ATX", default_title=True)
            modified_content = re.sub(r'(?<!^)(```)', r'\n\1', markdown, flags=re.MULTILINE)
            markdown_outputs.append(modified_content)

    # Concatenate all markdown outputs with a newline
    final_markdown = '\n\n'.join(markdown_outputs)
    return final_markdown

def remove_consecutive_empty_lines(text):
    # Remove consecutive empty lines, leaving only single empty lines
    return re.sub(r'\n\s*\n', '\n\n', text)


def save_to_file(file_name, content):
    # Save the content into the specified file
    with open(file_name, 'w', encoding='utf-8') as file:
        file.write(content)

# Test the function
# url = "https://docs.opencv.org/4.x/"
# root = "https://docs.opencv.org/4.x/"
# ROS
url = "https://wiki.ros.org/ROS/Tutorials/"
root = "https://wiki.ros.org/ROS/Tutorials/"
root_regex = r"^https://wiki.ros.org/ROS/Tutorials/"
root_filename = "ROS"
content_tags = [
    ('div', {'id': 'page', 'lang': 'en', 'dir': 'ltr'}),
]

# opencv
# url = "https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html"
# root_regex = r"^https://docs.opencv.org/4.x\/\w+\/\w+\/tutorial_py"
# root = "https://docs.opencv.org/4.x/d6/d00/"
# root_filename = "opencv"
# content_tags = [
#     ('div', {'class': 'contents'})
# ]

# url = "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/"
# root = "https://emanual.robotis.com/docs/en/platform/turtlebot3/"
# root_regex = r"^https://emanual.robotis.com/docs/en/platform/turtlebot3/"
# root_filename = "turtlebot3"
# content_tags = [
#     ('div', {'class': 'archive', 'id': 'archive'}),
# ]
delay = get_crawl_delay(cd_home(url))
links = extract_unique_links(url, root, root_regex, root_filename, content_tags, delay)

# inputs links, dir_name, delay
# process_links_and_save(links, root_filename, delay)
