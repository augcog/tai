import string
from rag.file_conversion_router.classes.chunk import Chunk
from rag.file_conversion_router.classes.page import Page
import tiktoken
import pickle
class VidPage(Page):
    def tree_segments_to_chunks(self):
        # seperate with recursive seperate
        print(len(self.tree_segments))
        for n, i in enumerate(self.tree_segments):
            content_chunks = self.recursive_separate(i['Segment_print'], 400)
            for count, content_chunk in enumerate(content_chunks):
                headers = i['Page_path']
                urls = [f"{self.page_url}&t={int(self.content['timestamp'][n])}" for header in headers]
                page_path = ' > '.join(f"{item} (h{i+1})" for i, item in enumerate(i['Page_path'])) + f" ({count})"
                self.chunks.append(Chunk(page_path, content_chunk, urls))
        return self.chunks



