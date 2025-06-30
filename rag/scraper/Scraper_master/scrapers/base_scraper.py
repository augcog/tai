from abc import ABC, abstractmethod
from typing import List

from scraper.Scraper_master.drivers.driver import Driver


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
