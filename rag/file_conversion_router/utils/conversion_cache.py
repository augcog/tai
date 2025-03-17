from concurrent.futures import Future
from pathlib import Path
from threading import Lock
from typing import Dict, List, Union, Optional
import pickle

from rag.file_conversion_router.utils.utils import load_conversion_version

"""
cache structure

ConversionCache._cache = {
    "d2a1e3f9...": {  # Hash as the key
        "input_path": "scrape_result/file1.txt",  # Last known filename
        "cache_paths": None,
        "conversion_version": "1.0",
        "last_updated": "2025-02-06 13:05:01"
    },
    "a4b2c3d1...": {
        "filename": "scrape_result/file2.txt",
        "cache_paths": ["archive/conversion_20250206-130501/file2.txt", "archive/conversion_20250206-130502/file2.txt"],
        "conversion_version": "1.0",
        "last_updated": "2025-02-05 20:45:10"
    }
}
"""


class ConversionCache:
    """A singleton class to handle caching of conversion results."""

    _instance = None  # Singleton instance
    _lock = Lock()  # Class-wide lock for thread safety

    _cache: Dict[str, Dict] = {}  # Persistent conversion cache
    _futures_cache: Dict[str, Future] = {}  # Temporary in-memory cache for futures
    _times_cache: Dict[str, float] = {}  # In-memory conversion times
    _access_count: Dict[str, int] = {}  # In-memory access counts
    _cache_file_path: Optional[Path] = None  # Cache file path
    version = load_conversion_version(Path(__file__).parent.parent / "conversion_version.txt")

    def __init__(self, *args, **kwargs):
        raise RuntimeError("ConversionCache is a singleton. Use ConversionCache.method() directly.")

    @classmethod
    def set_cache_path(cls, cache_file_path: Optional[Union[str, Path]]):
        """Initialize the cache and load from file if specified."""
        cls._cache_file_path = Path(cache_file_path) if cache_file_path else None
        if cls._cache_file_path:
            cls._load_cache()

    @classmethod
    def _load_cache(cls) -> None:
        """Load the cache from a pickle file if it exists."""
        with cls._lock:
            if cls._cache_file_path and cls._cache_file_path.exists():
                try:
                    with cls._cache_file_path.open("rb") as f:
                        cls._cache = pickle.load(f)
                except Exception as e:
                    print(f"Warning: Failed to load cache from {cls._cache_file_path}: {e}")
                    cls._cache = {}

    @classmethod
    def _save_cache(cls) -> None:
        """Persist the cache to a pickle file."""
        with cls._lock:
            if cls._cache_file_path:
                try:
                    print(f"Saving cache to {cls._cache_file_path}")
                    with cls._cache_file_path.open("wb") as f:
                        pickle.dump(cls._cache, f)
                except Exception as e:
                    print(f"Warning: Failed to save cache to {cls._cache_file_path}: {e}")

    @classmethod
    def get_cached_paths(cls, file_hash: str) -> Optional[List[str]]:
        """Retrieve cached paths for the given file hash."""
        with cls._lock:
            if file_hash in cls._cache:
                cls._access_count[file_hash] = cls._access_count.get(file_hash, 0) + 1
                return cls._cache[file_hash].get("cache_paths")
            return None

    @classmethod
    def get_file_conversion_version(cls, file_hash: str) -> Optional[str]:
        """Return the conversion version for a given file hash."""
        with cls._lock:
            return cls._cache[file_hash]["conversion_version"] if file_hash in cls._cache else None

    @classmethod
    def set_cache_and_time(cls, file_hash: str, input_path: str, paths: List[Path], time_taken: float) -> None:
        """Store cached conversion results and record the conversion time."""
        with cls._lock:
            str_paths = [str(p) for p in paths]
            cls._cache[file_hash] = {
                "cache_paths": str_paths,
                "input_path": input_path,
                "conversion_version": cls.version,
                "last_updated": str(paths[0].stat().st_mtime),
            }
            cls._times_cache[file_hash] = time_taken
            cls._access_count[file_hash] = 0
        cls._save_cache()  # Persist the cache if needed

    @classmethod
    def get_cached_time(cls, file_hash: str) -> Optional[float]:
        """Return the cached conversion time for a given file hash."""
        with cls._lock:
            return cls._times_cache.get(file_hash)

    @classmethod
    def get_access_count(cls, file_hash: str) -> int:
        """Return the number of times a cached item has been accessed."""
        with cls._lock:
            return cls._access_count.get(file_hash, 0)

    @classmethod
    def get_future(cls, file_hash: str) -> Optional[Future]:
        """Retrieve an in-progress conversion future for the given file hash."""
        with cls._lock:
            return cls._futures_cache.get(file_hash)

    @classmethod
    def store_future(cls, file_hash: str, future: Future) -> None:
        """Store a future object for an in-progress conversion."""
        with cls._lock:
            cls._futures_cache[file_hash] = future

    @classmethod
    def clear_future(cls, file_hash: str) -> None:
        """Remove a completed conversion future from the cache."""
        with cls._lock:
            cls._futures_cache.pop(file_hash, None)

    @classmethod
    def check_version(cls, file_hash: str) -> bool:
        with cls._lock:
            return cls._cache[file_hash]["conversion_version"] == cls.version

    @classmethod
    def calc_total_savings(cls) -> float:
        """Calculate the total time saved by using cached results."""
        with cls._lock:
            return sum(
                time * accesses
                for file_hash, time in cls._times_cache.items()
                if (accesses := cls._access_count.get(file_hash, 0)) > 0
            )


