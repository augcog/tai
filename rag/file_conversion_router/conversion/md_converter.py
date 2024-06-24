from pathlib import Path
import yaml
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page
from rag.file_conversion_router.classes.chunk import Chunk

class MarkdownConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override

    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        """Perform Markdown to Page conversion."""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        parent = input_path.parent
        stem = input_path.stem
        filetype = input_path.suffix.split(".")[1]
        with open(input_path, "r") as input_file:
            text = input_file.read()
        metadata = parent / (stem+"_metadata.yaml")
        with open(metadata, "r") as metadata_file:
            metadata_content = yaml.safe_load(metadata_file)
        url = metadata_content["URL"]
        page = Page(pagename=stem, content={'text': text}, filetype=filetype, page_url=url)
        return page

# converter = MarkdownConverter()
# converter._to_page(Path("/home/bot/roarai/rag/scraper/Scraper_master/opencv/tutorial_py_table_of_contents_calib3d/tutorial_py_root/tutorial_py_root.md"), Path("/home/bot/roarai/rag/scraper/Scraper_master/"))
