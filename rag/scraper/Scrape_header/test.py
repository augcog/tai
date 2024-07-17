import requests
from bs4 import BeautifulSoup
from markdownify import MarkdownConverter


def fetch_html(url):
    """Fetches HTML content from a URL."""
    try:
        response = requests.get(url)
        response.raise_for_status()  # Ensures we notice bad responses
        return response.text
    except requests.RequestException as e:
        print(f"Error fetching URL {url}: {e}")
        return None


def convert_html_to_soup(html_content):
    """Converts HTML content to a BeautifulSoup object."""
    return BeautifulSoup(html_content, 'html.parser')


def convert_soup_to_md(soup, **options):
    """Converts a BeautifulSoup object to Markdown."""
    converter = MarkdownConverter(**options)
    return converter.convert_soup(soup)


# Specify the URL of the website you want to convert
url = 'https://docs.opencv.org/4.x/d0/de3/tutorial_py_intro.html'

# Fetch HTML content from the website
html_content = fetch_html(url)

if html_content:
    # Convert the HTML to a BeautifulSoup object
    soup = convert_html_to_soup(html_content)

    # Convert the BeautifulSoup object to Markdown
    markdown_result = convert_soup_to_md(soup, heading_style="ATX")

    # Print or save the resulting Markdown content
    print(markdown_result)
else:
    print("Failed to retrieve HTML content.")
