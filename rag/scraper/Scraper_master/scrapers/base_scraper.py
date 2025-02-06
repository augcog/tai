from abc import ABC, abstractmethod

class BaseScraper(ABC):
    """
    Base abstract scraper.
    """
    @abstractmethod
    def scrape(self):
        """
        Execute the scraping task.
        """
        pass