import requests
from bs4 import BeautifulSoup
import re
from markdownify import markdownify as md

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

# Example usage
content_tags = [
    ('div', {'id': 'page', 'lang': 'en', 'dir': 'ltr'}),
]
print(html_to_markdown("https://wiki.ros.org/ROS/Tutorials/", content_tags))
