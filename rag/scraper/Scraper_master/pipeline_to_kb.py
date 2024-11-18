import requests
from bs4 import BeautifulSoup
import os
import time
import re
import yaml
from pathlib import Path
from typing import Union
import ffmpeg


from rag.scraper.Scraper_master.scrape_header import run_tasks, ScrapeHeader, match_tags
from rag.file_conversion_router.api import convert_directory
from rag.file_conversion_router.embedding_create import embedding_create


def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
def pipeline(yaml):
    """A pipeline to scrape, chunk and embedd websites into a knowledge base.
    A YAML file is used to specify the tasks to be performed. The YAML should be structured as follows:
    root_folder : "path/to/root/folder"
    tasks :
      - name : "Local Folder"
        local: True
        url : "https://website.url"
        root: "/root folder"
      - name : "/root folder"
        local: False
        url : "https://website.url"
        root: "https://website.url"

    """
    data = load_yaml(yaml)
    root1 = data['root_folder']
    print("ROOT1:",root1)
    root=os.path.abspath(root1)
    _, n= os.path.split(root)
    embedding_name=os.path.basename(os.path.normpath(root))
    markdown_path = root + "_md"
    # print("MDPATH", markdown_path)
    for task in data['tasks']:
        name = task['name']
        is_local = task['local']
        url = task['url']
        if is_local:
            print("NAME:",name)
            name = os.path.abspath(name)
            print("NAME:",name)
        else:
            print("NAME:",name)
            name = root + '/' + name
            if not os.path.exists(name):
                os.makedirs(name)
            print("NAME:",name)
            print(f"The task '{name}' is not local.")
            url=task['url']
            base_url=task['root']
            base_regex = rf"^{base_url}"
            content_tags = match_tags(url)
            scrapper = ScrapeHeader(url, base_url, base_regex, name, content_tags)
            scrapper.scrape()

        
    convert_directory(root, markdown_path)

    folder_name = "embedding"
    model = "BGE"
    embedding_create(markdown_path, n, embedding_name, folder_name, model)


def convert_only(yaml):
    data = load_yaml(yaml)
    root1 = data['root_folder']
    print("ROOT1:", root1)
    root = os.path.abspath(root1)
    _, n = os.path.split(root)
    embedding_name = os.path.basename(os.path.normpath(root))
    markdown_path = root + "_md"
    # print("MDPATH", markdown_path)
    # convert_directory(root, markdown_path)
    folder_name = "embedding"
    model = "BGE"
    embedding_create(markdown_path, n, embedding_name, folder_name, model)



if __name__ == "__main__":
    pipeline('task.yaml')
