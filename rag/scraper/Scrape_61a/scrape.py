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

def main():
    """
    The main function that initializes the parameters for website crawling and scraping.
    It sets up the initial URL, the root URL, the regex pattern for allowed URLs, the root directory name for saving files, and the content tags to search for.
    It then starts the scraping process by calling extract_unique_links.
    """

    # cs61a website
    url = "https://cs61a.org/"
    root = "https://cs61a.org/"
    root_regex = r"^https://cs61a.org/"
    root_filename = "cs61a_website_ver2"
    content_tags = [('section', {'id': 'calendar', 'class': 'table', 'cellpadding': '5px'}), ('div', {'class': 'col-md-9'})]
    delay = 0
    
    extract_unique_links(url, root, root_regex, root_filename, content_tags, 0, delay)
    #create_and_enter_dir("test")
    #cur_dir = os.getcwd()
    #print(cur_dir)
    #file_path = download_pdf("https://cs61a.org/exam/fa23/mt2/61a-fa23-mt2.pdf", cur_dir)
    #print(file_path)
    #pdf_to_md(file_path, cur_dir)


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
    # Remove trailing slash
    if link[-1] == '/':
        link = link[:-1]

    return link


def create_and_enter_dir(directory_name):
    """
    Creates a directory with the given name and enters it.
    - directory_name (str): The name of the directory to be created and entered.
    """
    # Create the directory if it doesn't exist
    if not os.path.exists(directory_name):
        os.mkdir(directory_name)
    
    # Change the current working directory
    os.chdir(directory_name)

def cd_home(url):
    """
    Returns the home directory of a given URL.
    - url (str): The URL whose home directory is needed.
    - Returns: The home directory URL.
    """
    return '/'.join(url.split('/')[0:3])

def get_crawl_delay(site_url, user_agent="*"):
    """
    Fetches the crawl delay from the robots.txt file of the given website.
    - site_url (str): The base URL of the website.
    - user_agent (str, optional): The user agent for which the crawl delay is requested. Defaults to '*'.
    - Returns: Crawl delay as specified in robots.txt, or 0 if not specified.
    """
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
        #filename = filename.split('.')[0]

        print("filename" + filename)

        if "proj" in link or "lab" in link or "hw" in link or "disc" in link:
            parts = link.split("/")
            folder = parts[-2]
            create_and_enter_dir(folder)

        if "_1pp" in link:
            continue
        
        print("last 4 char" + filename[-4:])
        if filename[-4:] == ".pdf":
            name = filename.split('.')[0]
            create_and_enter_dir(name)
            cur_dir = os.getcwd()
            file_path = download_pdf(link, cur_dir)

            #pdf_to_md(file_path, cur_dir)

            current_directory = os.getcwd()
            parent_directory = os.path.dirname(current_directory)
            os.chdir(parent_directory)
        elif filename[-4:] == ".zip":
            continue;
        else: 
            filename = filename.split('.')[0]
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
        
        if "proj" in link or "lab" in link or "hw" in link or "disc" in link:
            current_directory = os.getcwd()
            parent_directory = os.path.dirname(current_directory)
            os.chdir(parent_directory)
        
        # markdown_result = html_to_markdown(link, content_tags)
        # if markdown_result == 1:
            # continue
        
        # cleaned_markdown = remove_consecutive_empty_lines(markdown_result)
        
        # cur_dir = os.getcwd()
        # create_and_enter_dir(filename)
        
        # save_to_file(f'{filename}.md', cleaned_markdown)

        # parser = MarkdownParser(f'{filename}')
        # parser.print_header_tree()
        # parser.print_segment()
        # parser.concat_print()
        
        # os.chdir(cur_dir)
        # time.sleep(delay)

def extract_unique_links(url, root, root_regex, root_filename, content_tags, layer, delay=0, found_links=[]):
    
    if (layer == 2):
        return;

    # print("extract_unique_links")
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
    # print(url)
    #print(soup.prettify())
    unique_links = set()  # Create an empty set to store unique links
    # print(soup)
    for link in soup.find_all('a'):
        href = link.get('href')
        href = remove_slash_and_hash(href)
        if href in found_links or not href:
            continue
        clean_href = ''
        if href.startswith('http://') or href.startswith('https://'):
            clean_href = remove_slash_and_hash(href)
        elif href and href.startswith('#'):
            continue
        elif href and href.startswith('/'):
            clean_href = (remove_slash_and_hash(urljoin(cd_home(root), href)))
        elif href and not (href.startswith('http://') or href.startswith('https://')) and '/' in href:
            clean_href = (remove_slash_and_hash(urljoin(root, href)))
        if re.match(root_regex, clean_href) and clean_href not in found_links:
            unique_links.add(clean_href)

    links = list(unique_links)
    print(links)
    if not links:
        return
    process_links_and_save(links, root_filename, delay, content_tags=content_tags)
    found_links.extend(links)
    cur_dir = os.getcwd()
    for link in links:
        remove_slash_and_hash(link)
        filename=link.split('/')[-1]
        filename=filename.split('.')[0]
        extract_unique_links(link, root, root_regex, filename, content_tags, layer+1, delay, found_links)
        os.chdir(cur_dir)

def html_to_markdown(url, content_tags):
    """
    Converts HTML content from a URL to markdown format.
    - url (str): URL to fetch HTML content from.
    - content_tags (list): Specific HTML tags to convert to markdown.
    - Returns: Converted markdown content.
    """
    try:
        response = requests.get(url)
        response.raise_for_status()  # Raise an exception for HTTP errors
    except requests.RequestException as e:
        print(f"Failed to retrieve the URL due to: {e}")
        return 1

    # Parse the HTML content
    soup = BeautifulSoup(response.text, 'html.parser')

    markdown_outputs = []
    if content_tags:
        for tag_type, tag_attr in content_tags:
            # Find the specific tags
            content = soup.find_all(tag_type, tag_attr)
            for item in content:
                # Convert each HTML item to Markdown
                markdown = md(str(item), heading_style="ATX", default_title=True)
                modified_content = re.sub(r'(?<!^)(```)', r'\n\1', markdown, flags=re.MULTILINE)
                markdown_outputs.append(modified_content)
        # print(markdown_outputs)
        # Concatenate all markdown outputs with a newline
        final_markdown = '\n\n'.join(markdown_outputs)
    else:
        final_markdown = md(str(soup), heading_style="ATX", default_title=True)
        modified_content = re.sub(r'(?<!^)(```)', r'\n\1', final_markdown, flags=re.MULTILINE)
        final_markdown = modified_content
    return final_markdown

def remove_consecutive_empty_lines(text):
    """
    Removes consecutive empty lines from a text, leaving only single empty lines.
    - text (str): Text to be processed.
    - Returns: Cleaned text with single empty lines.
    """
    # Remove consecutive empty lines, leaving only single empty lines
    return re.sub(r'\n\s*\n', '\n\n', text)


def save_to_file(file_name, content):
    """
    Saves content to a file with the specified file name.
    - file_name (str): The name of the file to save the content.
    - content (str): The content to be saved.
    """
    # Save the content into the specified file
    with open(file_name, 'w', encoding='utf-8') as file:
        file.write(content)


def download_pdf(url, directory):
    """
    Downloads the pdf file at the url
    """
    filename = url.split("/")[-1]
    file_path = os.path.join(directory, filename)
    response = requests.get(url)
    if response.status_code == 200:
        with open(file_path, 'wb') as f:
            f.write(response.content)
        print(f"PDF downloaded successfully to: {file_path}")
    else:
        print(f"Failed to download PDF. Status code: {response.status_code}")
    return file_path

def pdf_to_md(file_path, folder_name):
    command = f"nougat {file_path} -o {folder_name}"
    os.system(command)


if __name__ == "__main__":
    main()