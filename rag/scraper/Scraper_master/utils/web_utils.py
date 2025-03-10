import os
import urllib.robotparser as robotparser
from bs4 import BeautifulSoup
from urllib.parse import urlparse, urlunparse, urljoin, unquote


def remove_slash_and_hash(link):
    """
    Removes trailing slash (if present) and hash fragment from a given URL.

    Parameters:
    - link (str): The URL from which the last segment should be modified.

    Returns:
    - str: The modified URL without the trailing slash and hash fragment.
    """
    if not link:
        return link
    link = link.split('#')[0]
    if not link:
        return link
    if link[-1] == '/':
        link = link[:-1]
    return link

def get_crawl_delay(site_url, user_agent="*"):
    """
    Fetches the crawl delay from the robots.txt file of the given website.

    Parameters:
    - site_url (str): The base URL of the website.
    - user_agent (str, optional): The user agent for which the crawl delay is requested. Defaults to '*'.

    Returns:
    - int: Crawl delay as specified in robots.txt, or 0 if not specified.
    """
    robots_url = site_url.rstrip('/') + '/robots.txt'
    rp = robotparser.RobotFileParser()
    rp.set_url(robots_url)
    try:
        rp.read()
        delay = rp.crawl_delay(user_agent)
        return delay if delay else 0
    except Exception as e:
        print(f"Error accessing or parsing robots.txt: {e}")
        return 0

def normalize_url(url: str) -> str:
    """
    Normalize a URL by removing default ports and trailing slashes in the path.
    """
    parsed = urlparse(url)

    # Determine if we should remove port
    # Default ports: 80 for HTTP, 443 for HTTPS
    if (parsed.scheme == 'http' and parsed.port == 80) or (parsed.scheme == 'https' and parsed.port == 443):
        netloc = parsed.hostname
    else:
        netloc = parsed.netloc

    # Remove trailing slash from the path if it's not the root
    path = parsed.path.rstrip('/')

    # Rebuild the URL
    normalized = urlunparse((parsed.scheme, netloc, path, parsed.params, parsed.query, parsed.fragment))
    return str(normalized)


def is_sub_path(root_url, test_url):
    normalized_root = normalize_url(root_url)
    normalized_test = normalize_url(test_url)
    return normalized_test.startswith(normalized_root)

def join_url(root_url: str, href: str) -> str:
    """
    Join a given href with a root_url to produce a fully qualified and normalized URL.

    Handles:
    - Absolute and relative URLs
    - Fragments (e.g. `#section`)
    - Special schemes (e.g., `mailto:`, `javascript:`) will be returned as-is,
      unless they are just anchors or empty.
    - If href is empty or None, returns the normalized root_url.
    """
    if not root_url:
        raise ValueError("root_url must be a non-empty absolute URL.")

    root_parsed = urlparse(root_url)
    if not root_parsed.scheme or not root_parsed.netloc:
        raise ValueError("root_url must be a valid absolute URL (with scheme and netloc).")

    # Handle empty or None href
    if not href or href.isspace() or href.lower().startswith(('#', 'javascript:', 'mailto:', 'tel:', 'data:')):
        return root_url

    # Use urljoin for general case: relative or absolute URLs
    joined = urljoin(root_url, href)

    # Normalize the result
    return joined

def extract_unique_links(true_url, html):
    """
    Extracts unique links from the given response.
    Parameters:
    - response (requests.Response): The response object to extract links from.
    Returns:
    - set: A set of unique links found in the response.
    """
    unique_links = set()
    soup = BeautifulSoup(html, 'html.parser')
    for link in soup.find_all('a'):
        href = link.get('href')
        link = join_url(true_url, href)
        if "www.youtube.com" not in link:
            link = normalize_url(link)
        unique_links.add(link)
    return unique_links

def get_file_name(url):
    MAX_FILENAME_LENGTH = 220  # Maximum length for a filename adjusted
    url = normalize_url(url)
    parsed_url = urlparse(url)
    filename = os.path.basename(os.path.normpath(parsed_url.path))
    if not parsed_url.path:
        filename = "root"
    filename = unquote(filename).replace(' ', '_')
    name, ext = os.path.splitext(filename)
    if len(filename) > MAX_FILENAME_LENGTH:
        truncated_name = name[:MAX_FILENAME_LENGTH - len(ext)]  # Adjust for extension length
        filename = f"{truncated_name}{ext}"
    return filename


if __name__ == "__main__":
    print(normalize_url("https://www.youtube.com/watch?v=IPec2A7j2bY&amp;list=PL6BsET-8jgYVCz97Y75GRXSWbb4sTpDIR"))