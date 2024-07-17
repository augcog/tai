from pathlib import Path
from rag.file_conversion_router.classes.page import Page
import yaml
import os
def to_page(input_path):
    filepath = Path(input_path).parent
    filename = Path(input_path).stem
    filetype = Path(input_path).suffix.split(".")[1]
    with open(input_path, "r") as input_file:
        content = input_file.read()
    metadata = filepath / (filename+"_metadata.yaml")
    with open(metadata, "r") as metadata_file:
        metadata_content = yaml.safe_load(metadata_file)
    url = metadata_content["URL"]
    page = Page(filename, content={'text': content}, filetype=filetype, page_url=url)
    page.page_seperate_to_segments()
    page.tree_print()
    page.tree_segments_to_chunks()



to_page("/home/bot/roarai/rag/scraper/Scraper_master/opencv/tutorial_py_table_of_contents_bindings/tutorial_py_bindings_basics/tutorial_py_bindings_basics.md")