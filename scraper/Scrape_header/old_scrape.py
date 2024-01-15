import requests
from bs4 import BeautifulSoup

def scrape_website(url):
    # Send a GET request to the website
    response = requests.get(url)
    
    # Parse the HTML content of the page with BeautifulSoup
    soup = BeautifulSoup(response.text, 'html.parser')
    
    # Find all the paragraph tags and get the text from each one
    paragraphs = soup.find_all('p')
    text = '\n'.join(paragraph.get_text() for paragraph in paragraphs)
    
    return text

from urllib.parse import urljoin
import os

if not os.path.exists('tree'):
    os.makedirs('tree')
visited_urls = set()
f = open('tree/output.txt', 'a')
def print_tree(url, base_url, depth=0, max_depth=2, prefix=''): 
    if depth > max_depth: 
        return 

    # Normalize the URL by removing trailing slashes
    normalized_url = url.rstrip('/')
    
    if normalized_url in visited_urls:
        return

    # Send a GET request to the website
    response = requests.get(url)

    # Parse the HTML content of the page with BeautifulSoup
    soup = BeautifulSoup(response.text, 'html.parser')

    # Find all the links on the page
    links = soup.find_all('a')

    # Add the current URL to the set of visited URLs
    visited_urls.add(normalized_url)
    
    # Write the URL to the output file
    with open('tree/output.txt', 'a') as f:
        f.write(f"{'--'*depth}{normalized_url}\n")

    for link in links:
        href = link.get('href')

        # Ignore None hrefs and those starting with a hashtag
        if href is None or href.startswith('#'): 
            continue 

        # Join the base url with the link href
        full_url = urljoin(url, href)

        # Only process URLs that start with the base URL
        if full_url.startswith(base_url):
            # Recursively print the tree structure for the current link
            print_tree(full_url, base_url, depth+1, max_depth, prefix + '--')

from termcolor import colored
from bs4 import BeautifulSoup, NavigableString, Tag

def print_html_tree(soup, depth=0):
    # If this is a root node, find the div element with class 'my-nav-content'
    if depth == 0:
        content_roots = soup.find_all('div', {'class': lambda value: value and 'content' in value})
        if content_roots:
            soup = content_roots[0]  # Set the root of the tree to be the first content root found
        else:
            print("No root element found with class containing 'content'")
            return

    colors = ['red', 'green', 'yellow', 'blue', 'magenta', 'cyan', 'white']
    color = colors[depth % len(colors)]  # Cycle through the colors

    if hasattr(soup, 'children'):
        texts = []
        for child in soup.children:
            if isinstance(child, Tag) and child.name == 'code':
                texts.append("<code>" + child.text + "</code>")
            elif isinstance(child, Tag) and child.name == 'a' and child.has_attr('href'):
                texts.append("<link>" + child.text + "</link>")
            elif isinstance(child, str):
                text = child.strip()
                if text:
                    texts.append(text)

        # Combine the collected texts and print once
        combined_text = ' '.join(texts)
        if combined_text:
            try:
                print(colored('-'*depth + combined_text, color))
            except UnicodeEncodeError:
                print(colored('-'*depth + combined_text.encode('cp1252', errors='ignore').decode('cp1252'), color))
        else:
            # If no combined text is formed, continue traversing children
            for child in soup.children:
                print_html_tree(child, depth+1)

    else:
        text = soup.strip()
        if text:
            try:
                print(colored('-'*depth + text, color))
            except UnicodeEncodeError:
                print(colored('-'*depth + text.encode('cp1252', errors='ignore').decode('cp1252'), color))










url = 'https://docs.ros.org/en/galactic/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html'
# url = 'https://numpy.org/install/'
response = requests.get(url)
soup = BeautifulSoup(response.text, 'html.parser')
pritify = soup.prettify()
print(pritify)
for i in range(10):
    print()
print_html_tree(soup)


# The URL of the website you want to scrape
url = 'https://docs.ros.org/en/galactic/index.html'
# Print the tree structure of the website
base_url = 'https://docs.ros.org/en/galactic/'
# print_tree(base_url, base_url, max_depth=5)
# Scrape the website
text = scrape_website(url)

# Write the text to a .txt file
with open('output.txt', 'w', encoding='utf-8') as f:
    f.write(text)

# import requests

# # Fetch the webpage
# response = requests.get('https://docs.ros.org/en/galactic/index.html')

# # Print the HTML content of the page
# print(response.text)
