import requests
from .driver import Driver, Resp
from .requests_driver import RequestsDriver
from bs4 import BeautifulSoup
import re
from scraper.Scraper_master.utils.cache import get_cache


class CachedRequestsDriver(RequestsDriver):
    """
    A cached version of RequestsDriver that caches HTTP responses
    to avoid redundant network requests.
    """
    
    def __init__(self, cache_ttl: int = 3600, enable_cache: bool = True):
        """
        Initialize the cached driver.
        
        Args:
            cache_ttl: Cache time-to-live in seconds (default: 1 hour)
            enable_cache: Whether to enable caching (useful for testing)
        """
        super().__init__()
        self.enable_cache = enable_cache
        self.cache_ttl = cache_ttl
        if enable_cache:
            self.cache = get_cache()

    def download_raw(self, filename, url: str):
        """
        Download content with caching support.
        
        First checks cache for existing response, if not found or expired,
        makes HTTP request and caches the result.
        """
        if not self.enable_cache:
            return super().download_raw(filename, url)
        
        # Try to get cached response
        cached_data = self.cache.get_cached_response(url)
        if cached_data:
            # Restore cached response
            content_type = cached_data['content_type']
            content = cached_data['content']
            true_url = cached_data['true_url']
            
            # Save file from cached content
            if "text/html" in content_type:
                soup = BeautifulSoup(content, 'html.parser')
                title = soup.title.string.strip() if soup.title and soup.title.string else None
                filename = (re.sub(r'\s+', ' ', re.sub(r'[\\/:"*?<>|]+', ' ', title)).strip() + '.html' 
                           if title else filename.rstrip('.html') + '.html')
                with open(filename, "w", encoding="utf-8") as file:
                    file.write(content)
            else:
                with open(filename, "wb") as file:
                    file.write(content)
            
            return filename, Resp(
                html_content=content if "text/html" in content_type else None,
                is_html="text/html" in content_type,
                true_url=true_url,
            )
        
        # Cache miss - make actual HTTP request
        response = requests.get(url, stream=True)
        response.raise_for_status()
        
        content_type = response.headers.get("Content-Type", "").lower()
        
        # Read content for caching
        if "text/html" in content_type:
            content = response.text
            # Parse HTML and get the title
            soup = BeautifulSoup(content, 'html.parser')
            title = soup.title.string.strip() if soup.title and soup.title.string else None
            filename = (re.sub(r'\s+', ' ', re.sub(r'[\\/:"*?<>|]+', ' ', title)).strip() + '.html' 
                       if title else filename.rstrip('.html') + '.html')
            with open(filename, "w", encoding="utf-8") as file:
                file.write(content)
        else:
            # For binary content, we need to read it all for caching
            content = b''
            for chunk in response.iter_content(chunk_size=8192):
                content += chunk
            with open(filename, "wb") as file:
                file.write(content)
        
        # Cache the response
        cache_data = {
            'content_type': content_type,
            'content': content,
            'true_url': response.url
        }
        self.cache.cache_response(url, cache_data, self.cache_ttl)
        
        return filename, Resp(
            html_content=content if "text/html" in content_type else None,
            is_html="text/html" in content_type,
            true_url=response.url,
        )

    def get_cache_stats(self):
        """Get cache statistics."""
        if not self.enable_cache:
            return {"caching": "disabled"}
        return self.cache.get_stats()

    def clear_cache(self):
        """Clear all cached responses."""
        if self.enable_cache:
            self.cache.clear("response")