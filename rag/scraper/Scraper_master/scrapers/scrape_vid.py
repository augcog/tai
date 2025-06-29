import os
import requests
import yaml
from bs4 import BeautifulSoup
from urllib.parse import urlparse, parse_qs
import yt_dlp

from rag.scraper.Scraper_master.scrapers.base_scraper import BaseScraper
from rag.scraper.Scraper_master.utils.file_utils import *


class VideoScraper(BaseScraper):
    def scrape(self, url, driver, task_folder_path):
        # Set base folder for all downloads
        os.makedirs(task_folder_path, exist_ok=True)
        if self._is_playlist(url):
            self._download_playlist(url, task_folder_path)
        else:
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

    def _is_playlist(self, url):
        """Returns True if the given URL is a playlist."""
        ydl_opts = {"quiet": True, "no_warnings": True}

        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            info = ydl.extract_info(
                url, download=False
            )  # Get metadata without downloading

        return info.get("_type") == "playlist"

    def _download_video(self, url, folder, index=None):
        """Downloads a single video and saves metadata."""
        video_info = self._get_video_info(url)
        if not video_info:
            print(f"Skipping {url}, unable to retrieve info.")
            return

        title = (
            video_info.get("title", "Untitled").replace("/", "_").replace("\\", "_")
            + f"_{index}"
        )
        video_folder = os.path.join(folder, title)

        # Download video
        self._download_yt_video(url, video_folder)

        # Save metadata
        self._save_metadata(video_folder + "/" + f"{title}_metadata.yaml", url)
        print(f"Downloaded Video: {title} --- {url}")

    def _download_playlist(self, url, base_folder):
        """Downloads all videos in a playlist and organizes them properly."""
        ydl_opts = {
            "quiet": True,
            "extract_flat": False,  # Force extraction of full video details
            "noplaylist": False,  # Explicitly allow playlist downloads
        }

        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            try:
                playlist_info = ydl.extract_info(url, download=False)
            except Exception:
                print(f"Skipping {url}, unable to retrieve playlist info.")
                return

        # Extract playlist title
        playlist_title = (
            playlist_info.get("title", "Untitled Playlist")
            .replace("/", "_")
            .replace("\\", "_")
        )
        playlist_folder = os.path.join(base_folder, playlist_title)
        os.makedirs(playlist_folder, exist_ok=True)
        # print(playlist_info)
        # Extract individual video URLs from the playlist
        if "entries" in playlist_info:
            for i, video in enumerate(playlist_info["entries"]):
                if video and "url" in video:
                    self._download_video(video["url"], playlist_folder, index=i)

        print(f"Downloaded playlist: {playlist_title} --- {url}")

    def _get_video_info(self, url):
        """Fetches video or playlist metadata using yt-dlp."""
        ydl_opts = {"quiet": True, "no_warnings": True, "noplaylist": True}
        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            try:
                return ydl.extract_info(url, download=False)
            except Exception:
                return None

    def _download_yt_video(self, url, folder):
        """Downloads a YouTube video using yt-dlp."""
        ydl_opts = {
            "outtmpl": os.path.join(folder, "%(title)s.%(ext)s"),
            "format": "best",
            "quiet": True,
            "noplaylist": True,
        }
        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            ydl.download([url])

    def _save_metadata(self, path, url):
        yaml_content = f"URL: {url}"
        save_to_file(path, yaml_content)


if __name__ == "__main__":
    url = "https://www.youtube.com/watch?v=1P2UgdAWwYg&list=PL6BsET-8jgYXTuSlJNYQS740YMCRHT79g&ab_channel=JohnDeNero"
    # import yt_dlp
    #
    # # video_url = "https://www.youtube.com/watch?v=VIDEO_ID"
    #
    # ydl_opts = {
    #     "outtmpl": "downloads/%(title)s.%(ext)s",
    # }
    #
    # with yt_dlp.YoutubeDL(ydl_opts) as ydl:
    #     ydl.download([url])
    #     import yt_dlp
    parsed_url = urlparse(url)
    query_params = parse_qs(parsed_url.query)
    print("list" in query_params and "playlist" in parsed_url.path)
