from abc import ABC, abstractmethod
class Chunk(ABC):
    def __init__(self, title, title_content, chunk_url):
        # dictionary of attributes
        self.title = title
        # file type (md, pdf, etc.)
        self.title_content = title_content
        # page url
        self.chunk_url = chunk_url


