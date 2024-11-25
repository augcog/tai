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
from dataclasses import dataclass, field
from typing import Dict, Any, List


@dataclass
class Chunk:
    """
    Represents a chunk of content with associated metadata and properties.

    Attributes:
        content: The main content text
        titles: Title or titles associated with the chunk
        chunk_url: URL source of the chunk
        metadata: Additional metadata as a dictionary
    """
    # TODO: Revise the Chunk design here. Is it necessary to have titles, and chunk_url as separate fields?
    #  Can they be combined into a single metadata field?
    content: str
    titles: str = "default_title"
    chunk_url: List[str] = ["default_no_url"],
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        """Ensure metadata is properly initialized."""
        if self.metadata is None:
            self.metadata = {}

        # Ensure core properties are included in metadata
        self.metadata.update({
            'titles': self.titles,
            'chunk_url': self.chunk_url,
            **self.metadata  # Keep any existing metadata
        })

    def __eq__(self, other):
        return self.titles == other.titles and self.content == other.content and self.chunk_url == other.chunk_url
        """
        Compare two chunks for equality.

        Args:
            other: Another Chunk instance to compare with

        Returns:
            bool: True if chunks are equal, False otherwise
        """
        if not isinstance(other, Chunk):
            return False

        return (
                self.titles == other.titles and
                self.content == other.content and
                self.chunk_url == other.chunk_url
        )

    def update_metadata(self, new_metadata: Dict[str, Any]) -> None:
        """
        Update chunk metadata safely.

        Args:
            new_metadata: Dictionary of new metadata to add or update
        """
        if self.metadata is None:
            self.metadata = {}
        self.metadata.update(new_metadata)

    def get_metadata(self, key: str, default: Any = None) -> Any:
        """
        Safely get metadata value by key.

        Args:
            key: Metadata key to retrieve
            default: Default value if key doesn't exist

        Returns:
            Value associated with key or default if not found
        """
        if self.metadata is None:
            return default
        return self.metadata.get(key, default)

    @property
    def core_metadata(self) -> Dict[str, Any]:
        """
        Get core metadata properties.

        Returns:
            Dictionary containing core metadata
        """
        return {
            'titles': self.titles,
            'chunk_url': self.chunk_url
        }
