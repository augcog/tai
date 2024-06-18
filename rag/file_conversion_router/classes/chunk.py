from abc import ABC, abstractmethod
class Chunk(ABC):
    def __init__(self, titles, content, chunk_url):
        # dictionary of attributes
        self.titles = titles
        # file type (md, pdf, etc.)
        self.content = content
        # page url
        self.chunk_url = chunk_url



