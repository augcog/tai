import requests
import re
from markdownify import markdownify as md

def html_to_markdown(url):
    # Fetch HTML content from the URL
    response = requests.get(url)
    response.raise_for_status()  # Raise an exception for HTTP errors

    # Convert the HTML to Markdown
    markdown = md(response.text, heading_style="ATX", default_title=True)

    return markdown

def remove_consecutive_empty_lines(text):
    # Remove consecutive empty lines, leaving only single empty lines
    return re.sub(r'\n\s*\n', '\n\n', text)

if __name__ == "__main__":
    # link = "http://wiki.ros.org/noetic"
    # link = "https://docs.ros.org/en/galactic/Tutorials.html"
    # link = "https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html"
    # link = "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/"
    # link = "https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/"
    link = "https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv"

    markdown_result = html_to_markdown(link)

    cleaned_markdown = remove_consecutive_empty_lines(markdown_result)

    # For demonstration purposes, print the result
    # Assuming cleaned_markdown is defined elsewhere in your code.
    with open('markdown.md', 'w') as file:
        file.write(cleaned_markdown)

