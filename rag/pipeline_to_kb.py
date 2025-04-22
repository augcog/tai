import os
import yaml

# from rag.scraper.Scraper_master.factory import ScraperFactory
# from rag.scraper.Scraper_master.configs import ScraperConfig
from rag.file_conversion_router.api import convert_directory
from rag.file_conversion_router.embedding_create import embedding_create
from rag.scraper.Scraper_master.scrapers.web_scraper import WebScraper

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
    cache_path = root + "_cache"
    log_path = os.path.abspath(data.get('log_path', f'{root}_log"'))

    # print("MDPATH", markdown_path)
    scraper = WebScraper(yaml)
    scraper.run()

    convert_directory(root, markdown_path, log_dir=log_path, cache_path=cache_path)

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
    pipeline('/home/bot/bot/tai/rag/scraper/Scraper_master/CS61A.yaml')
