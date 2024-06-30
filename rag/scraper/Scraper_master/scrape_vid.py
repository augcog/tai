from rag.scraper.Scraper_master.base_scraper import BaseScraper
from pytube import Playlist, YouTube
import os
from utils import save_to_file
class ScrapeVid(BaseScraper):
    def __init__(self, url, root_filename):
        super().__init__(url)
        self.root_filename = root_filename

    def get_playlist_urls(self, playlist_url):
        playlist = Playlist(playlist_url)
        # Retrieve videos and their titles
        videos_with_titles = [(video, video.title) for video in playlist.videos]

        # Sanitize titles and sort videos by these titles
        videos_with_titles.sort(key=lambda x: "".join(char for char in x[1] if char.isalnum() or char in " -_").strip())

        # Return only the URLs, now sorted by the sanitized title
        return [video.watch_url for video, _ in videos_with_titles]
    def content_extract(self, filename, url, **kwargs):
        pass

    def metadata_extract(self, filename, url, **kwargs):
        yaml_content = f"URL: {url}"
        save_to_file(f'{filename}', yaml_content)

    def scrape(self):
        video_urls = self.get_playlist_urls(self.url)
        for url in video_urls:
            video = YouTube(url)
            stream = video.streams.get_highest_resolution()

            # Create a unique folder for each video based on its title
            safe_title = "".join(x for x in video.title if x.isalnum() or x in " -_").replace(" ", "_").strip()
            video_path = os.path.join(self.root_filename, safe_title)
            os.makedirs(video_path, exist_ok=True)

            download_filename = stream.download(output_path=video_path)
            os.rename(download_filename, os.path.join(video_path, safe_title + ".mp4"))
            metadata_filename = os.path.join(video_path, safe_title + "_metadata.yml")
            self.metadata_extract(metadata_filename, url)

base_path = 'Denero_videos'
os.makedirs(base_path, exist_ok=True)
playlist_url = 'https://www.youtube.com/watch?v=31EDjrN1x5k&list=PL6BsET-8jgYUA8ryM_zeRA3H_RAMNBrN3&ab_channel=JohnDeNero'
scraper = ScrapeVid(playlist_url, base_path)
scraper.scrape()
# download_videos(video_urls, base_path)