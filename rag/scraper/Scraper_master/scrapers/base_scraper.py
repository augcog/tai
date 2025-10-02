from abc import ABC, abstractmethod
from typing import List

from scraper.Scraper_master.drivers.driver import Driver
from scraper.Scraper_master.utils.file_utils import save_to_file


class BaseScraper(ABC):
    """
    Base abstract scraper.
    """

    @abstractmethod
    def scrape(self, url: str, driver: Driver, task_folder_path: str) -> List:
        """
        Scrape the url using corresponding driver.
        Return the normalized links found in the web page the url directed to.
        """
        raise NotImplementedError("This method should be overridden by subclasses.")

    def _save_metadata(self, filename, url):
        yaml_content = f"URL: {url}"
        save_to_file(f"{filename}_metadata.yaml", yaml_content)
