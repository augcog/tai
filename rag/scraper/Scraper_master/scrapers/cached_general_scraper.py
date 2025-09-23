import os
from urllib.parse import urlparse
from scraper.Scraper_master.scrapers.general_scraper import GeneralScraper
from scraper.Scraper_master.utils.file_utils import *
from scraper.Scraper_master.utils.web_utils import *
from scraper.Scraper_master.utils.cache import get_cache


class CachedGeneralScraper(GeneralScraper):
    """
    A cached version of GeneralScraper that caches extracted links
    to avoid redundant link extraction operations.
    """
    
    def __init__(self, cache_ttl: int = 1800, enable_cache: bool = True):
        """
        Initialize the cached scraper.
        
        Args:
            cache_ttl: Cache time-to-live in seconds (default: 30 minutes)
            enable_cache: Whether to enable caching
        """
        super().__init__()
        self.enable_cache = enable_cache
        self.cache_ttl = cache_ttl
        if enable_cache:
            self.cache = get_cache()

    def scrape(self, url, driver, task_folder_path):
        """
        Scrape with link caching support.
        
        First checks cache for extracted links, if not found,
        performs extraction and caches the result.
        """
        if not self.enable_cache:
            return super().scrape(url, driver, task_folder_path)
        
        # Create cache key that includes the task folder path to handle different contexts
        cache_key = f"{url}:{task_folder_path}"
        
        # Try to get cached links
        cached_links = self.cache.get_cached_links(cache_key)
        if cached_links is not None:
            # Still need to create directory and download file if it doesn't exist
            dir_path = os.path.join(task_folder_path, urlparse(url).path.lstrip("/").rsplit("/", 1)[0])
            create_and_enter_dir(dir_path)
            filename = get_file_name(url)
            
            # Check if file already exists, if not download it
            if not os.path.exists(filename):
                filename, resp = driver.download_raw(filename, url)
                self._save_metadata(filename, url)
            
            return cached_links
        
        # Cache miss - perform normal scraping
        create_and_enter_dir(
            os.path.join(task_folder_path, urlparse(url).path.lstrip("/").rsplit("/", 1)[0])
        )
        filename = get_file_name(url)
        filename, resp = driver.download_raw(filename, url)
        self._save_metadata(filename, url)
        
        links = []
        if resp.is_html:
            links = extract_unique_links(resp.true_url, resp.html_content)
            # Cache the extracted links
            self.cache.cache_links(cache_key, links, self.cache_ttl)
        
        return links

    def get_cache_stats(self):
        """Get cache statistics."""
        if not self.enable_cache:
            return {"caching": "disabled"}
        return self.cache.get_stats()

    def clear_cache(self):
        """Clear all cached links."""
        if self.enable_cache:
            self.cache.clear("links")