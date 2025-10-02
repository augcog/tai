import hashlib
import json
import os
import pickle
import time
from pathlib import Path
from typing import Any, Optional, Union
import threading


class ScraperCache:
    """
    A flexible caching system for the scraper that supports:
    - HTTP response caching
    - Link extraction caching
    - Metadata caching
    - Configurable TTL and size limits
    """

    def __init__(self, cache_dir: str = ".scraper_cache", default_ttl: int = 3600, max_size: int = 1000):
        """
        Initialize the cache.
        
        Args:
            cache_dir: Directory to store cache files
            default_ttl: Default time-to-live in seconds (1 hour)
            max_size: Maximum number of cached items
        """
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(exist_ok=True)
        self.default_ttl = default_ttl
        self.max_size = max_size
        self.metadata_file = self.cache_dir / "cache_metadata.json"
        self._lock = threading.Lock()
        self._load_metadata()

    def _load_metadata(self):
        """Load cache metadata from file."""
        try:
            if self.metadata_file.exists():
                with open(self.metadata_file, 'r') as f:
                    self.metadata = json.load(f)
            else:
                self.metadata = {}
        except (json.JSONDecodeError, IOError):
            self.metadata = {}

    def _save_metadata(self):
        """Save cache metadata to file."""
        try:
            with open(self.metadata_file, 'w') as f:
                json.dump(self.metadata, f, indent=2)
        except IOError:
            pass

    def _get_cache_key(self, key: str, cache_type: str = "default") -> str:
        """Generate a cache key hash."""
        full_key = f"{cache_type}:{key}"
        return hashlib.md5(full_key.encode()).hexdigest()

    def _get_cache_file(self, cache_key: str) -> Path:
        """Get the cache file path for a given key."""
        return self.cache_dir / f"{cache_key}.cache"

    def _is_expired(self, cache_key: str) -> bool:
        """Check if a cache entry is expired."""
        if cache_key not in self.metadata:
            return True
        
        created_time = self.metadata[cache_key].get('created_time', 0)
        ttl = self.metadata[cache_key].get('ttl', self.default_ttl)
        return time.time() - created_time > ttl

    def _cleanup_expired(self):
        """Remove expired cache entries."""
        expired_keys = []
        for cache_key, meta in self.metadata.items():
            if self._is_expired(cache_key):
                expired_keys.append(cache_key)
        
        for cache_key in expired_keys:
            self._remove_cache_entry(cache_key)

    def _enforce_size_limit(self):
        """Enforce cache size limit by removing oldest entries."""
        if len(self.metadata) <= self.max_size:
            return
        
        # Sort by creation time and remove oldest entries
        sorted_entries = sorted(
            self.metadata.items(),
            key=lambda x: x[1].get('created_time', 0)
        )
        
        entries_to_remove = len(self.metadata) - self.max_size
        for cache_key, _ in sorted_entries[:entries_to_remove]:
            self._remove_cache_entry(cache_key)

    def _remove_cache_entry(self, cache_key: str):
        """Remove a cache entry and its file."""
        if cache_key in self.metadata:
            del self.metadata[cache_key]
        
        cache_file = self._get_cache_file(cache_key)
        if cache_file.exists():
            try:
                cache_file.unlink()
            except OSError:
                pass

    def get(self, key: str, cache_type: str = "default") -> Optional[Any]:
        """
        Get a value from cache.
        
        Args:
            key: The cache key
            cache_type: Type of cache (e.g., 'response', 'links', 'metadata')
            
        Returns:
            Cached value or None if not found/expired
        """
        with self._lock:
            cache_key = self._get_cache_key(key, cache_type)
            
            if self._is_expired(cache_key):
                self._remove_cache_entry(cache_key)
                return None
            
            cache_file = self._get_cache_file(cache_key)
            if not cache_file.exists():
                if cache_key in self.metadata:
                    del self.metadata[cache_key]
                return None
            
            try:
                with open(cache_file, 'rb') as f:
                    return pickle.load(f)
            except (pickle.PickleError, IOError):
                self._remove_cache_entry(cache_key)
                return None

    def set(self, key: str, value: Any, cache_type: str = "default", ttl: Optional[int] = None):
        """
        Set a value in cache.
        
        Args:
            key: The cache key
            value: The value to cache
            cache_type: Type of cache (e.g., 'response', 'links', 'metadata')
            ttl: Time-to-live in seconds (uses default_ttl if None)
        """
        with self._lock:
            cache_key = self._get_cache_key(key, cache_type)
            cache_file = self._get_cache_file(cache_key)
            
            try:
                with open(cache_file, 'wb') as f:
                    pickle.dump(value, f)
                
                self.metadata[cache_key] = {
                    'key': key,
                    'cache_type': cache_type,
                    'created_time': time.time(),
                    'ttl': ttl or self.default_ttl
                }
                
                self._cleanup_expired()
                self._enforce_size_limit()
                self._save_metadata()
                
            except (pickle.PickleError, IOError):
                if cache_file.exists():
                    cache_file.unlink()

    def cache_response(self, url: str, response_data: dict, ttl: Optional[int] = None):
        """Cache an HTTP response."""
        self.set(url, response_data, "response", ttl)

    def get_cached_response(self, url: str) -> Optional[dict]:
        """Get a cached HTTP response."""
        return self.get(url, "response")

    def cache_links(self, url: str, links: list, ttl: Optional[int] = None):
        """Cache extracted links for a URL."""
        self.set(url, links, "links", ttl)

    def get_cached_links(self, url: str) -> Optional[list]:
        """Get cached links for a URL."""
        return self.get(url, "links")

    def cache_metadata(self, key: str, metadata: dict, ttl: Optional[int] = None):
        """Cache metadata."""
        self.set(key, metadata, "metadata", ttl)

    def get_cached_metadata(self, key: str) -> Optional[dict]:
        """Get cached metadata."""
        return self.get(key, "metadata")

    def clear(self, cache_type: Optional[str] = None):
        """
        Clear cache entries.
        
        Args:
            cache_type: If specified, only clear entries of this type
        """
        with self._lock:
            if cache_type is None:
                # Clear all cache
                for cache_file in self.cache_dir.glob("*.cache"):
                    cache_file.unlink()
                self.metadata.clear()
            else:
                # Clear specific cache type
                keys_to_remove = []
                for cache_key, meta in self.metadata.items():
                    if meta.get('cache_type') == cache_type:
                        keys_to_remove.append(cache_key)
                
                for cache_key in keys_to_remove:
                    self._remove_cache_entry(cache_key)
            
            self._save_metadata()

    def get_stats(self) -> dict:
        """Get cache statistics."""
        with self._lock:
            self._cleanup_expired()
            
            stats = {
                'total_entries': len(self.metadata),
                'cache_types': {},
                'cache_dir_size_mb': 0
            }
            
            # Count by cache type
            for meta in self.metadata.values():
                cache_type = meta.get('cache_type', 'unknown')
                stats['cache_types'][cache_type] = stats['cache_types'].get(cache_type, 0) + 1
            
            # Calculate directory size
            total_size = 0
            for cache_file in self.cache_dir.glob("*"):
                if cache_file.is_file():
                    total_size += cache_file.stat().st_size
            
            stats['cache_dir_size_mb'] = round(total_size / (1024 * 1024), 2)
            
            return stats


# Global cache instance
_cache_instance = None


def get_cache(cache_dir: str = ".scraper_cache", default_ttl: int = 3600, max_size: int = 1000) -> ScraperCache:
    """Get or create a global cache instance."""
    global _cache_instance
    if _cache_instance is None:
        _cache_instance = ScraperCache(cache_dir, default_ttl, max_size)
    return _cache_instance