import os
from urllib.parse import parse_qs, urlparse

import requests
import yaml
import yt_dlp
from bs4 import BeautifulSoup

from scraper.Scraper_master.scrapers.base_scraper import BaseScraper
from scraper.Scraper_master.utils.file_utils import *


class VideoScraper(BaseScraper):
    def scrape(self, url, driver, task_folder_path):
        # Set base folder for all downloads
        os.makedirs(task_folder_path, exist_ok=True)
        self._download_video(url, task_folder_path)
        return []

    def _extract_youtube_links(self, soup):
        """Extracts YouTube video and playlist links from anchor tags in HTML."""
        links = []
        for a_tag in soup.find_all("a", href=True):
            url = a_tag["href"]
            if self._is_youtube_url(url):
                links.append(url)
        return links

    def _is_youtube_url(self, url):
        """Checks if a URL belongs to YouTube."""
        parsed_url = urlparse(url)
        return parsed_url.netloc in [
            "www.youtube.com",
            "youtube.com",
            "m.youtube.com",
            "youtu.be",
        ]

    def _download_video(self, url, folder):
        """Downloads a single video and saves metadata."""
        outtmpl = os.path.join(folder, "./%(playlist_title&{}|)s/%(playlist_index&{}-|)s%(title)s.%(ext)s")

        ydl_opts = {"quiet": True,
                    "no_warnings": True,
                    # "cookiefile": "/home/bot/bot/yk/YK_final/www.youtube.com_cookies.txt",
                    "cookiesfrombrowser": ('chrome', ),
                    "outtmpl": outtmpl,
                    "ignoreerrors": True,
                    "sleep_interval": 5,
                    "max_sleep_interval": 10,
                    }
        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            video_info = ydl.extract_info(url, download=True)
        if not video_info:
            print(f"Skipping {url}, unable to retrieve info.")
            return
        if video_info.get('_type') == 'playlist':
            for entry in video_info['entries']:
                video_url = entry.get('original_url', entry.get('url'))
                filepath = f"{entry['requested_downloads'][0]['filepath']}"
                self._save_metadata(filepath, video_url)
        else:
            # For single video
            video_url = video_info.get('original_url', video_info.get('url'))
            filepath = f"{video_info['requested_downloads'][0]['filepath']}"
            self._save_metadata(filepath, video_url)


if __name__ == "__main__":
    url = "https://www.youtube.com/watch?v=1P2UgdAWwYg&list=PL6BsET-8jgYXTuSlJNYQS740YMCRHT79g&ab_channel=JohnDeNero"
    parsed_url = urlparse(url)
    query_params = parse_qs(parsed_url.query)
    print("list" in query_params and "playlist" in parsed_url.path)
