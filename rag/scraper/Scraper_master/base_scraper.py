from abc import ABC, abstractmethod

class BaseScraper(ABC):
    def __init__(self, url):
        self.url=url

    # tranverse through the entire website, playlist etc.
    def scrape(self) -> None:
        pass

    @abstractmethod
    # input url, save it into a file
    def content_extract(self, filename, url, **kwargs):
        pass

    # input url, extract information
    def metadata_extract(self, filename, url, **kwargs):
        pass