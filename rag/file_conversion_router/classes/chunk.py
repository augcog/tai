from abc import ABC, abstractmethod
class Chunk(ABC):
    def __init__(self, titles, content, chunk_url, page_num):
        # dictionary of attributes
        self.titles = titles
        # file type (md, pdf, etc.)
        self.content = content
        # page url
        self.chunk_url = chunk_url
        # page number
        self.page_num = page_num

