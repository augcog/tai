from dataclasses import dataclass, field
from typing import Dict, Any, List, Union


@dataclass
class Chunk:
    """
    Represents a chunk of content with associated metadata and properties.

    Attributes:
        content: The main content text
        titles: Title or titles associated with the chunk
        chunk_url: URL source of the chunk
    """
    content: str
    titles: str = "default_title"
    chunk_url: Union[str, List[str]] = ("default_no_url",)
    is_split: bool = False
    index: Any = None

    def __repr__(self):
        snippet = (self.content[:80] + "…") if len(self.content) > 80 else self.content
        return (
            f"Chunk(index={self.index}, "
            f"titles={self.titles!r}, "
            f"is_split={self.is_split}, "
            f"url={self.chunk_url!r}, "
            f"content='{snippet}')"
        )
