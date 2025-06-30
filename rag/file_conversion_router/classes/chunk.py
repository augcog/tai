from dataclasses import dataclass, field
from typing import Any, Dict, List


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
    chunk_url: str = ("default_no_url",)
    metadata: Dict[str, Any] = field(default_factory=dict)
    page_num: Any = None

    def __post_init__(self):
        if not isinstance(self.metadata, dict):
            raise TypeError(
                f"metadata must be a dictionary, got {type(self.metadata).__name__}"
            )
        self.metadata.update(
            {
                "titles": self.titles,
                "chunk_url": self.chunk_url,
                "page_num": self.page_num,
            }
        )

    def __eq__(self, other):
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
            self.titles == other.titles
            and self.content == other.content
            and self.chunk_url == other.chunk_url
            and self.page_num == other.page_num
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
            "titles": self.titles,
            "chunk_url": self.chunk_url,
            "page_num": self.page_num,
        }
