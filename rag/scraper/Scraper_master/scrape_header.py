import requests
from bs4 import BeautifulSoup
import os
import time
import re
from termcolor import colored
from urllib.parse import urljoin
from markdownify import markdownify as md
from rag.scraper.Scraper_master.base_scraper import BaseScraper
import yaml

from utils import create_and_enter_dir, delete_and_exit_dir, remove_consecutive_empty_lines, save_to_file,remove_slash_and_hash, cd_home,get_crawl_delay


content_tags_dict = {
    "https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html": [('div', {'class': 'contents'})],
    "https://www.berkeley.edu/": [],
    "https://wiki.ros.org/ROS/Tutorials/": [('div', {'id': 'page', 'lang': 'en', 'dir': 'ltr'})],
    "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/": [('div', {'class': 'archive', 'id': 'archive'})],
    "https://platform.openai.com/docs/introduction": [('div', {'class': 'docs-body'})],
    "https://cloud.google.com/docs/": [('main', {'role': 'main', 'class': 'devsite-main-content'})],
    "https://docs.qualcomm.com/bundle/publicresource/topics/80-62010-1/Overview.html?product=Windows%20on%20Snapdragon": [('div', {'text': 'This section defines contents of documents', 'class': 'contentmodule ng-star-inserted'})],
    "https://www.svlsimulator.com/docs/": [('div', {'role': 'main'})],
    "https://cs61a.org/": [('section', {'id': 'calendar', 'class': 'table', 'cellpadding': '5px'}), ('div', {'class': 'col-md-9'})],
    "https://ucb-ee106.github.io/106b-sp23site/": [('main', {'class': 'main-content'})],
    "https://en.wikipedia.org/wiki/University_of_California,_Berkeley": [('main', {'id': 'content', 'class': 'mw-body'})],
    "https://freesociologybooks.com/Sociology_Of_The_Family/01_Changes_and_Definitions.php#google_vignette": [('div',{'id':'content','class':'clearfix'})],
    "https://leginfo.legislature.ca.gov/faces/codes.xhtml": [('div', {'class':'tab_content'})],
    "https://guide.berkeley.edu/courses/":[],
}

def match_tags(url):
    """
    Matches the URL with suitable content tags for scraping
    Parameters:
    - url (str): The URL to match content tags with
    Returns: Content tags for the given URL
    """
    if url in content_tags_dict:
        return content_tags_dict[url]
    return []

class ScrapeHeader(BaseScraper):
    def __init__(self, url, root, root_regex, root_filename, content_tags):
        super().__init__(url)  # Assuming BaseScraper requires the URL, you might need to adjust based on actual BaseScraper's constructor.
        self.url = url
        self.root = root
        self.root_regex = root_regex
        self.root_filename = root_filename
        self.content_tags = content_tags
        self.delay = get_crawl_delay(cd_home(url))
        self.http_header = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'}
        
    def process_links_and_save(self, links, dir_name, delay, content_tags):
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
            cur_dir = os.getcwd()
            create_and_enter_dir(filename)
            # if not os.path.exists(filename):
            #     os.makedirs(filename, exist_ok=True)
            error = self.content_extract(filename, link, content_tags=content_tags)
            # print("error", error)
            if error == 1:
                print("error",filename)
                delete_and_exit_dir()
                continue
            self.metadata_extract(filename, link)
            os.chdir(cur_dir)
            time.sleep(delay)



    def extract_unique_links(self, url, root, root_regex, root_filename, content_tags, delay=0, found_links=[]):
        print("extract_unique_links")
        """
        Extract and print unique links from a given URL that start with a specified root.
    
        Parameters:
        - url (str): The URL from which links are to be extracted.
        - root (str): The root URL which extracted links should start with to be considered.
    
        Returns:
        - list: A list of unique links that match the criteria.
        """
        reqs = requests.get(url, headers=self.http_header)
        soup = BeautifulSoup(reqs.text, 'html.parser')
        #print(soup)
        unique_links = set()  # Create an empty set to store unique links
        for link in soup.find_all('a'):
            href = link.get('href')
            href = remove_slash_and_hash(href)
            print(href)
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
        self.process_links_and_save(links, root_filename, delay, content_tags=content_tags)
        found_links.extend(links)
        cur_dir = os.getcwd()
        for link in links:
            remove_slash_and_hash(link)
            filename = link.split('/')[-1]
            filename = filename.split('.')[0]
            self.extract_unique_links(link, root, root_regex, filename, content_tags, delay, found_links)
            os.chdir(cur_dir)


    def html_to_markdown(self, url, content_tags):
        """
        Converts HTML content from a URL to markdown format.
        - url (str): URL to fetch HTML content from.
        - content_tags (list): Specific HTML tags to convert to markdown.
        - Returns: Converted markdown content.
        """
        try:
            response = requests.get(url, headers=self.http_header)
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
            # Concatenate all markdown outputs with a newline
            final_markdown = '\n\n'.join(markdown_outputs)
        else:
            final_markdown = md(str(soup), heading_style="ATX", default_title=True)
            modified_content = re.sub(r'(?<!^)(```)', r'\n\1', final_markdown, flags=re.MULTILINE)
            final_markdown = modified_content
        return final_markdown
    
    def download_pdf(self, url, filename):
        """
        Downloads the PDF file at the url.
        Parameters:
        - url (str): The URL to download the PDF from
        - filename(str): Name of the PDF file
        Returns: 1 on failure, file path on success.
        """
        file_path = os.path.join(os.getcwd(), filename + ".pdf")
        response = requests.get(url, headers=self.http_header)
        if response.status_code == 200:
            with open(file_path, 'wb') as f:
                f.write(response.content)
                print(f"PDF downloaded successfully to: {file_path}")
                return file_path
        else:
            print(f"Failed to download file. HTTP Status Code: {response.status_code}")
            return 1
    
    # Override
    def content_extract(self, filename, url, **kwargs):
        if ".pdf" in url:
            pdf_result = self.download_pdf(url, filename)
            return pdf_result
        else:
            content_tags=kwargs['content_tags']
            markdown_result = self.html_to_markdown(url, content_tags)
            if markdown_result != 1:
                cleaned_markdown = remove_consecutive_empty_lines(markdown_result)
                print("saving file...")
                save_to_file(f'{filename}.md', cleaned_markdown)
            return markdown_result

    def metadata_extract(self, filename, url, **kwargs):
        yaml_content = f"URL: {url}"
        save_to_file(f'{filename}_metadata.yaml', yaml_content)

    def scrape(self):
            self.extract_unique_links(self.url,self.root,self.root_regex,self.root_filename,self.content_tags, self.delay)

def run_tasks(yaml_file):
    with open(yaml_file, 'r') as file:
        configuration=yaml.safe_load(file)
        root=configuration['root_folder']
        root=os.path.abspath(root)
        for task in configuration['tasks']:
            url=task['url']
            base_url = url.split('/')
            base_url = '/'.join(base_url[:3]) + '/'
            base_regex = rf"^{base_url}"
            root_folder = root + '/' + task['name']
            content_tags = match_tags(url)

            scrapper = ScrapeHeader(url, base_url, base_regex, root_folder, content_tags)
            scrapper.scrape()

if __name__ == "__main__":
    # url =  "https://guide.berkeley.edu/courses/"
    # root_regex = r"^https://classes.berkeley.edu/"
    # root =  "https://classes.berkeley.edu/"
    # root_filename = "courses"
    # #
    # content_tags = match_tags(url)
    #
    # scrapper = ScrapeHeader(url, root, root_regex, root_filename, content_tags)
    # scrapper.scrape()
    run_tasks('106b_task.yaml')
