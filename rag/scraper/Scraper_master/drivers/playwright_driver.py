from playwright.sync_api import sync_playwright
import requests
from pathlib import Path

from .driver import Driver, Resp


class PlaywrightDriver(Driver):
    def __init__(self, headless=True):
        self._play = None
        self._browser = None
        self._context = None
        self._page = None
        self._headless = headless
        self._initialize_browser()

    def _initialize_browser(self):
        self._play = sync_playwright().start()
        self._browser = self._play.chromium.launch(headless=self._headless)
        self._context = self._browser.new_context(accept_downloads=True)
        self._page = self._context.new_page()

    def download_raw(self, filename: str, url: str) -> Resp:
        """
        Attempt to download the content as HTML first, and fallback to binary if an error occurs.

        Parameters:
        - filename (str): Path to save the content.
        - url (str): The URL to fetch.

        Returns:
        - Resp: An object containing HTML content (if applicable), whether it is HTML, and the final URL.
        """
        response = requests.get(url, stream=True)
        response.raise_for_status()
        content_type = response.headers.get("content-type", "").lower()
        if "text/html" in content_type:
            resp = self._page.goto(url)
            html_content = self._page.content()
            with open(f"{filename.split('.')[0]}.html", "w", encoding="utf-8") as f:
                f.write(html_content)
            return Resp(
                html_content=html_content,
                is_html=True,
                true_url=resp.url,
            )
        else:
            with open(filename, "wb") as file:
                for chunk in response.iter_content(chunk_size=8192):
                    file.write(chunk)
            return Resp(
                html_content=None,
                is_html=False,
                true_url=response.url,
            )

    def close(self):
        if self._page:
            self._page.close()
        if self._context:
            self._context.close()
        if self._browser:
            self._browser.close()
        if self._play:
            self._play.stop()
