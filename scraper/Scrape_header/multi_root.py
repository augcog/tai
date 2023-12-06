from bs4 import BeautifulSoup
import requests

def fetch_title_structure(url):
    # Fetch the HTML content using requests
    response = requests.get(url)
    response.raise_for_status()  # Raise an exception for HTTP errors

    soup = BeautifulSoup(response.text, 'html.parser')

    # Define the tags we're interested in
    tags_of_interest = ['title', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6']

    current_content = []
    
    skip_tags = []  # To keep track of tags we're skipping (e.g., nested tags inside <pre>)

    for tag in soup.find_all(True):  # Iterating over all tags
        if skip_tags and tag in skip_tags:  # Skip over nested tags that we want to ignore
            continue

        if tag.name == 'pre':  # Handling code blocks
            code = tag.get_text()  # Get the content of the code block preserving whitespace
            current_content.append(code)
            skip_tags.extend(tag.find_all(True))  # Add all nested tags inside <pre> to skip_tags
            continue

        if tag.name in tags_of_interest:  # If the tag is one of our headers/titles
            if current_content:  # If there's any content accumulated so far
                print('\n'.join(current_content))  # Print the accumulated content
                print('-' * 40)  # Separator
                current_content = []  # Clear the content

            current_content.append(tag.get_text(strip=True))  # Add the header/title text to the content
        else:
            # If the tag has text content, add it to the current content list
            text = tag.get_text(strip=True)
            if text:
                current_content.append(text)
                
    if current_content:  # Printing any remaining content
        print('\n'.join(current_content))

# Test
url = "https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html"
fetch_title_structure(url)
