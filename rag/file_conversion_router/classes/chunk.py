from abc import ABC, abstractmethod
class Chunk(ABC):
    def __init__(self, titles, content, chunk_url, page_num=None):
        # dictionary of attributes
        self.titles = titles
        # file type (md, pdf, etc.)
        self.content = content
        # page url
        self.chunk_url = chunk_url
        # page number
        self.page_num = page_num

    def __eq__(self, other):
        return self.titles == other.titles and self.content == other.content and self.chunk_url == other.chunk_url
