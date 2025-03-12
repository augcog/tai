from bs4 import BeautifulSoup

from rag.scraper.Scraper_master.scrapers.base_scraper import BaseScraper
from rag.scraper.Scraper_master.utils.web_utils import *
from rag.scraper.Scraper_master.utils.file_utils import *

from rag.scraper.Scraper_master.drivers.playwright_driver import PlaywrightDriver
from rag.scraper.Scraper_master.drivers.requests_driver import RequestsDriver


class GeneralScraper(BaseScraper):
    def scrape(self, url, driver, task_folder_path):
        create_and_enter_dir(os.path.join(task_folder_path, urlparse(url).path.lstrip('/')))
        filename = get_file_name(url)
        resp = driver.download_raw(filename, url)
        self._save_metadata(filename, url)
        links = []
        if resp.is_html:
            links = extract_unique_links(resp.true_url, resp.html_content)
        return links

    def _save_metadata(self, filename, url):
        base_filename, _ = os.path.splitext(filename)
        yaml_content = f"URL: {url}"
        # Save file without .pdf in the filename
        save_to_file(f'{base_filename}_metadata.yaml', yaml_content)