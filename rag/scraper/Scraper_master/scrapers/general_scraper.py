from scraper.Scraper_master.scrapers.base_scraper import BaseScraper
from scraper.Scraper_master.utils.file_utils import *
from scraper.Scraper_master.utils.web_utils import *


class GeneralScraper(BaseScraper):
    def scrape(self, url, driver, task_folder_path):
        create_and_enter_dir(
            os.path.join(task_folder_path, urlparse(url).path.lstrip("/").rsplit("/", 1)[0])
        )
        filename = get_file_name(url)
        filename,resp = driver.download_raw(filename, url)
        self._save_metadata(filename, url)
        links = []
        if resp.is_html:
            links = extract_unique_links(resp.true_url, resp.html_content)
        return links

    def _save_metadata(self, filename, url):
        file_type = filename.split(".")[-1]
        if file_type =='pdf':
            url = re.sub(r'#page=\d+$', '', url)
        yaml_content = f"URL: {url}"
        save_to_file(f"{filename}_metadata.yaml", yaml_content)