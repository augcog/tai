import requests
from bs4 import BeautifulSoup
import os
import time
import re
import yaml
from pathlib import Path
from typing import Union


from rag.scraper.Scraper_master.scrape_header import run_tasks
from rag.file_conversion_router.api import convert_directory
from rag.file_conversion_router.embedding_create import embedding_create

def pipeline(tasks: Union[str, Path]):
    """A pipeline to scrape, chunk and embedd websites into a knowledge base.
    A YAML file is used to specify the tasks to be performed. The YAML should be structured as follows:
    root_folder : "path/to/root/folder"
    tasks :
      - name : "Website Name"
        url : "https://website.url"
      - name : "Website Name"
        url : "https://website.url"

    """
    with open(tasks, 'r') as file:
        configuration=yaml.safe_load(file)
        print("CONFIGURATION:",configuration)
        root1=configuration['root_folder']
        print("ROOT1:",root1)
        root=os.path.abspath(root1)
        embedding_name=os.path.basename(os.path.normpath(root))
        print("EMBEDDING NAME",embedding_name)
        print("ROOT2:",root)

        run_tasks(tasks)
        print("ROOT:",root)
        markdown_path = root+"_md"
        print("MDPATH",markdown_path)

        convert_directory(root,markdown_path)
        name=configuration['tasks'][0]['name']
        embedding_name=os.path.basename(os.path.normpath(root))
        print(embedding_name)
        folder_name="embedding"
        model="BGE"
        embedding_create(markdown_path, name, embedding_name, folder_name, model)

if __name__ == "__main__":
    pipeline('106b_task.yaml')


