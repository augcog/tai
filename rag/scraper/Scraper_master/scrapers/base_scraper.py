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

    def _save_metadata(self, filename, url, additional_metadata=None):
        """
        Save metadata for a file.

        Args:
            filename: Path to the file (without _metadata.yaml extension)
            url: Primary URL for the content
            additional_metadata: Optional dict with additional metadata fields
        """
        yaml_content = f"URL: {url}\n"
        if additional_metadata:
            for key, value in additional_metadata.items():
                if value is not None:
                    yaml_content += f"{key}: {value}\n"
        save_to_file(f"{filename}_metadata.yaml", yaml_content)
