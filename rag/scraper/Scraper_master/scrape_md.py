import requests
import base64
import yaml
import os
import json
from rag.scraper.Scraper_master.base_scraper import BaseScraper
from utils import create_and_enter_dir, cd_back_link, replace_backslash_with_slash, save_to_file


class ScrapeMd(BaseScraper):
    def __init__(self, url, root_filename):
        super().__init__(url)
        self.root_filename = root_filename

    def get_content(self, url):
        """
        Retrieves content from the given URL
        - url (str): URL to fetch the content from.
        - Returns: The content fetched from the URL.
        """
        # Fetch the content from the URL
        headers = {'Accept': 'application/json'}
        response = requests.get(url, headers=headers)
        data = response.json()
        content = data['payload']['blob']['rawLines']

        # Convert the content list into a single string
        content = '\n'.join(content)

        return content


    def fetch_file_content_from_github(self, url):
        """
        Fetches the file content from a GitHub URL.
        - url (str): The GitHub URL from which the file content is to be fetched.
        - Returns: The decoded content of the file.
        """
        response = requests.get(url)
        data = response.json()
        content = base64.b64decode(data['content']).decode('utf-8')
        return content


    def get_url_child(self, url):
        """
        Fetches child markdown file names from a given GitHub directory URL.
        - url (str): The GitHub directory URL to fetch the markdown files from.
        - Returns: A list of markdown file names.
        """
        headers = {'Accept': 'application/json'}
        response = requests.get(url, headers=headers)
        data = response.json()
        childs = []
        md_files = data["payload"]["tree"]["items"]
        for i in md_files:
            name = i['name']
            if name.endswith('.md'):
                childs.append(name)
        return childs


    def fetch_urls(self, base_url, nav):
        """
        Recursively fetches URLs and their content based on the navigation structure defined in the mkdocs.yml file.
        - base_url (str): The base URL for the mkdocs documentation.
        - nav (list): A list or dictionary defining the navigation structure from mkdocs.yml.
        """
        print(f"nav: {nav}")
        # nav = [list(i.values())[0] if isinstance(i, dict) else i for i in nav]
        for i in nav:
            cur_dir = os.getcwd()
            if isinstance(i, str) and i.endswith(".md"):
                filename = i.split("/")[-1]
                url = os.path.join(base_url, i)
                url += "?plain=1"
                self.content_extract(filename, url)
                self.metadata_extract(filename, url)
                os.chdir(cur_dir)
                continue
            key, value = i.popitem()
            if (not key):
                continue
            create_and_enter_dir(key)
            # if it is nested
            if isinstance(value, list):
                self.fetch_urls(base_url, value)
            # if it is not md
            elif value.endswith('/') and not value.startswith('http'):
                cur_child_dir = os.getcwd()
                url = os.path.join(base_url, value)
                childs = self.get_url_child(url)
                print(childs)
                for child in childs:
                    filename = value.split("/")[-1]
                    create_and_enter_dir(child.replace('.md', ''))
                    child_url = os.path.join(url, child)
                    child_url += "?plain=1"
                    filename = child
                    self.content_extract(filename, child_url)
                    self.metadata_extract(filename, child_url)
                    os.chdir(cur_child_dir)

            elif not value.endswith('.md'):
                continue
            else:
                url = os.path.join(base_url, value)
                url += "?plain=1"
                filename = value.split("/")[-1]
                self.content_extract(filename, url)
                self.metadata_extract(filename, url)

            os.chdir(cur_dir)






    def extract_yaml_sections(self, data: str) -> str:
        """
        Extract specific sections from the provided YAML data string.

        :param data: A string containing the YAML data.
        :return: A string containing the extracted sections.
        """
        lines = data.split("\n")

        result = []
        for i in range(len(lines)):
            stripped_line = lines[i].strip()
            # Check if the line starts with one of our desired keys
            if stripped_line.startswith(("repo_url:", "edit_uri:", "docs_dir:")):
                result.append(lines[i])
            elif stripped_line.startswith("nav:"):
                result.append(lines[i])
                cur = i + 1
                while lines[cur].strip().startswith("-") or lines[cur].strip() == ("") or lines[cur].strip().startswith("#"):
                    result.append(lines[cur])
                    cur += 1
                    if cur >= len(lines):
                        break
                break
            # If in capture mode, add the line to the result

        # Combine the result lines back into a string
        output = "\n".join(result)

        return output

    def scrape(self) -> None:
        create_and_enter_dir(self.root_filename)
        # Set headers for the request to specify that application/json is accepted
        headers = {'Accept': 'application/json'}

        # Send the GET request
        response = requests.get(self.url, headers=headers)

        # Load the JSON data returned by the server
        data = response.json()

        # Extract specific content, assumed to be raw YAML lines
        content = data['payload']['blob']['rawLines']
        content = '\n'.join(content)

        # Function to extract YAML sections, assuming it's defined elsewhere
        content = self.extract_yaml_sections(content)

        # Load the YAML data
        parsed_yaml = yaml.load(content, Loader=yaml.SafeLoader)

        # Extract various pieces of data from the YAML
        repo_url = parsed_yaml['repo_url']
        edit_url = parsed_yaml.get('edit_uri')
        if edit_url:
            edit_url = edit_url.replace('\\', '/').replace('edit/', 'blob/')

        # Determine the base URL using a function cd_back_link assumed to be defined elsewhere
        docs_dir = parsed_yaml.get('docs_dir', None)
        if docs_dir:
            base_url = os.path.join(cd_back_link(self.url), docs_dir)
        else:
            base_url = os.path.join(cd_back_link(self.url), "docs/")
        base_url = replace_backslash_with_slash(base_url)

        # Fetch URLs based on navigation data
        nav = parsed_yaml['nav']
        self.fetch_urls(base_url, nav)
    def content_extract(self, filename, url, **kwargs):
        markdown = self.get_content(url)
        save_to_file(filename, markdown)

    def metadata_extract(self, filename, url, **kwargs):
        yaml_content = f"URL: {url}"
        save_to_file(f'{filename}_metadata.yaml', yaml_content)

if __name__ == '__main__':
    root_filename = 'MonashDataFluency'

    github_url = "https://github.com/MonashDataFluency/python-web-scraping/blob/master/mkdocs.yml"
    ScrapeMd(github_url, root_filename).scrape()

