from file_conversion_router.classes.chunk import Chunk
from file_conversion_router.classes.page import Page


class VidPage(Page):
    def tree_segments_to_chunks(self):
        for n, segment in enumerate(self.tree_segments):
            default_title = (
                segment["Page_path"][-1] if segment["Page_path"] else "(NO TITLE)"
            )
            splitted_contents = self.recursive_separate(segment["Segment_print"], 400)
            is_split_flag = len(splitted_contents) > 1
            if is_split_flag:
                merged_title = f"{default_title}"
            else:
                merged_title = default_title
            for count, content_chunk in enumerate(splitted_contents):
                headers = segment["Page_path"]
                timestamp_list = self.content['timestamp']
                timestamp = timestamp_list[n]
            # TODO: only add start time and add url as separate field self.url and self.chunk_index
                if len(self.page_url) > 0 and timestamp is not None:
                    url_with_time = f"{self.page_url}&t={float(timestamp)}"
                else:
                    url_with_time = " "
                self.chunks.append(
                    Chunk(
                        content=content_chunk,
                        titles=merged_title,
                        chunk_url=url_with_time,
                        page_num=segment.get("page_num", None),
                        is_split=is_split_flag,
                    )
                )
        return self.chunks
