from bs4 import BeautifulSoup

from rag.scraper.Scraper_master.scrapers.base_scraper import BaseScraper
from rag.scraper.Scraper_master.utils.web_utils import *
from rag.scraper.Scraper_master.utils.file_utils import *
from rag.scraper.Scraper_master.logger import logger, set_up_logger


class GeneralScraper(BaseScraper):
    def __init__(self, driver, config):
        self.driver = driver
        self.config = config
        self.max_depth = 15
        self.visited = set()
        self.task_folder_path = os.path.abspath(config.root_folder + '/' + config.name)
        self.logger = logger
        set_up_logger(self.logger, config.log_folder + '/' + config.name + '.log')

    def scrape(self):
        start_url = self.config.start_url
        if not start_url:
            raise ValueError("No 'start_url' defined in config.")
        self.logger.info(f"Scraping {start_url} with {self.driver.__class__.__name__}")
        self._dfs_scrape(normalize_url(start_url), 0)
        self.driver.close()

    def _dfs_scrape(self, url, depth):
        if depth > self.max_depth:
            return
        if url in self.visited:
            return
        if depth != 0 and not is_sub_path(self.config.base_url, url):
            return

        indent = '         ' * depth
        self.logger.info(f"")
        self.logger.info(f"{indent}Downloading files in url:")
        self.logger.info(f"{indent}{url}")

        self.visited.add(url)
        try:
            create_and_enter_dir(os.path.join(self.task_folder_path, urlparse(url).path.lstrip('/')))
            filename = get_file_name(url)
            resp = self.driver.download_raw(filename, url)
            self._save_metadata(filename, url)
        except Exception as ex:
            self.logger.error(f"{indent}Error processing link {url}: {ex}")
            delete_and_exit_dir()
            return
        if resp.is_html:
            links = extract_unique_links(resp.true_url, resp.html_content)
            for link in links:
                self._dfs_scrape(link, depth + 1)

    def _save_metadata(self, filename, url):
        yaml_content = f"URL: {url}"
        save_to_file(f'{filename}_metadata.yaml', yaml_content)