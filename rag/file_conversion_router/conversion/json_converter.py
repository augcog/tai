from pathlib import Path
import yaml
import json
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page
from rag.file_conversion_router.classes.chunk import Chunk
from rag.scraper.Scrape_ed.scrape import scrape_json
from rag.scraper.Scrape_ed.filter import json_kb_filter

class JsonConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        """Perform Json to Markdown conversion.

        Arguments:
        input_path -- Path to the input json file.
        output_folder -- Path to the folder where the output md file will be saved.
        """
        # call on scrape.py's modified json converter + load the json_data
        output_path = output_path.with_suffix(".md")
        with open(input_path, 'r') as file:
            json_data = json.load(file)
        
        # filter the json so only good data remains -- comment out if filter not needed/wanted!
        json_data = json_kb_filter(json_data)

        # run the ed scraper
        scrape_json(json_data, output_path)
        return output_path
    
    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        """Perform Markdown to Page conversion."""
        try:
            input_path = self._to_markdown(input_path, output_path)
        except Exception as e:
            self._logger.error(f"An error occurred during markdown conversion: {str(e)}")
            raise

        output_path.parent.mkdir(parents=True, exist_ok=True)

        filetype = input_path.suffix.lstrip('.')
        with open(input_path, "r") as input_file:
            text = input_file.read()

        metadata_path = input_path.with_name(f"{input_path.stem}_metadata.yaml")
        metadata_content = self._read_metadata(metadata_path)
        url = metadata_content.get("URL")
        return Page(pagename=input_path.stem, content={'text': text}, filetype=filetype, page_url=url)

