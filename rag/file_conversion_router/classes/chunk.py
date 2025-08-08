from dataclasses import dataclass, field
from typing import Dict, Any, List, Union, Optional


@dataclass
class Chunk:
    """
    Represents a chunk of content with associated metadata and properties.

    Attributes:
        content: The main content text
        titles: Title or titles associated with the chunk
        chunk_url: URL source of the chunk
        is_split: Indicates if the chunk is split from a larger content
        index: Index or identifier for the chunk
        file_path: Optional file path where the chunk is stored
    """
    content: str
    titles: str = "default_title"
    chunk_url: Union[str, List[str]] = ("default_no_url",)
    is_split: bool = False
    index: Any = None
    file_path: Optional[str] = None

    def __repr__(self):
        snippet = (self.content[:80] + "…") if len(self.content) > 80 else self.content
        return (
            f"Chunk_index={self.index}, "
            f"titles={self.titles!r}, "
            f"is_split={self.is_split}, "
            f"url={self.chunk_url!r}, "
            f"content='{snippet}', "  # Fixed: Added missing comma
            f"file_path={self.file_path!r}"
        )