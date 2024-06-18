import requests
import os
import re
from urllib.parse import urljoin


global parser
from rag.scraper.Scraper_master.base_scraper import BaseScraper
from utils import cd_back_link, create_and_enter_dir, save_to_file
ignore = ["glossary", "*"]

class ScrapeRst(BaseScraper):
    def __init__(self, github_url, doc_url, filename):
        super().__init__(github_url)
        self.filename = filename
        self.doc_url = doc_url
    def get_content(self, url):
        """
        Fetch data from a given URL, save it to a file, and print certain parts of the JSON.

        :param url: The URL to fetch data from.
        :return: None
        """
        print(f"Fetching data from {url}")
        headers = {'Accept': 'application/json'}
        response = requests.get(url+'?plain=1', headers=headers)
        data = response.json()

        # Saving the entire fetched JSON to a file
        data = data['payload']['blob']['rawLines']
        data = '\n'.join(data)
        return data



    def extract_toctree_from_rst(self, url):
        """
        Extracts the table of contents tree (toctree) from a reStructuredText (.rst) file.

        Args:
            filename (str): Path to the .rst file to parse.

        Returns:
            list: A list containing the extracted toctree links.
        """
        content = self.get_content(url)
        toctree_content = []
        lines = content.split('\n')
        for i in range(len(lines)):
            line = lines[i]
            if line.startswith(".. toctree::"):
                cur = i + 1
                while cur < len(lines) and lines[cur].strip() != "":
                    cur += 1

                cur += 1
                while cur < len(lines) and lines[cur].strip() != "":
                    match = re.search(r'<(.*?)>', lines[cur])
                    if match:
                        toctree_content.append(match.group(1).strip())
                    else:
                        toctree_content.append(lines[cur].strip())
                    cur += 1
            else:
                i += 1
        # print(f"toctree:")
        # for link in toctree_content:
        #     print(link)
        return toctree_content

    def tree_call(self, cur_file, url, home_url, home_dir):
        """
        Recursively navigates through the file structure of a website and fetches content based on RST files.
        - cur_file (str): The current file being processed.
        - url (str): The URL of the current file.
        - home_url (str): The base URL of the website.
        - home_dir (str): The base directory on the local filesystem.
        """
        filename = f"{cur_file}"
        self.content_extract(filename, url)
        self.metadata_extract(filename, url)
        toctree_content = self.extract_toctree_from_rst(url)
        url = cd_back_link(url) + '/'
        current_directory = os.getcwd()

        for sublink in toctree_content:
            if sublink.startswith('/'):
                sublink = sublink[1:]
                part = sublink.split("/")
                cur_name, dir = part[-1], '/'.join(part[:-1])
                if cur_name == '*' or re.match(r'^https.*', sublink) is not None:
                    continue
                os.chdir(home_dir)
                create_and_enter_dir(dir)
                sublink = sublink[:-4] if sublink.endswith('.rst') else sublink
                temp_url = url
                url = home_url + sublink + ".rst"
                # print(url)
                self.tree_call(cur_name, url, home_url, home_dir)
                url = temp_url
                os.chdir(current_directory)

            else:
                part = sublink.split("/")
                cur_name, dir = part[-1], '/'.join(part[:-1])
                if cur_name in ignore or re.match(r'^https.*', sublink) is not None:
                    continue
                create_and_enter_dir(dir)
                sublink = sublink[:-4] if sublink.endswith('.rst') else sublink
                url += sublink + ".rst"
                self.tree_call(cur_name, url, home_url, home_dir)
                url = cd_back_link(url, len(part)) + '/'
                os.chdir(current_directory)

    def scrape(self):
        create_and_enter_dir(self.filename)
        home_url = self.url.rsplit('/', 1)[0]
        # Current directory
        home_dir = os.getcwd()
        self.tree_call('index', self.url, home_url, home_dir)
    def content_extract(self, filename, url, **kwargs):
        """
        Extracts content from a given URL and saves it to a file.
        - filename (str): The name of the file to save the content to.
        - url (str): The URL to fetch the content from.
        """
        content = self.get_content(url)
        save_to_file(f"{filename}.rst", content)

    def metadata_extract(self, filename, url, **kwargs):
        yaml_content = f"URL: {self.doc_url.replace('index.html', '')}{'/'.join(url.split('/')[url.split('/').index('master') + 1:]).replace('.rst', '.html')}"
        save_to_file(f'{filename}_metadata.yaml', yaml_content)

if __name__ == "__main__":
    filename = "Moveit"
    # doc_url = "https://moveit.picknik.ai/main/index.html"
    # github_url = f"https://github.com/ros-planning/moveit_tutorials/blob/main/index.rst?plain=1"
    doc_url = "https://moveit.github.io/moveit_tutorials/index.html"
    github_url = "https://github.com/moveit/moveit_tutorials/blob/master/index.rst"
    ScrapeRst(github_url, doc_url, filename).scrape()
