from rag.scraper.Scraper_master.scrapers.general_scraper import GeneralScraper
from rag.scraper.Scraper_master.drivers.requests_driver import RequestsDriver
from rag.scraper.Scraper_master.drivers.playwright_driver import PlaywrightDriver
from rag.scraper.Scraper_master.configs import GeneralTaskConfig, RstTaskConfig


class ScraperFactory:
    @staticmethod
    def create_scraper(task_config):
        """
        Create a scraper instance for the given task_config.
        Use global_config for root_folder or other global settings.
        """
        if isinstance(task_config, GeneralTaskConfig):
            # Decide driver
            if task_config.driver_type == "requests":
                driver = RequestsDriver()
            elif task_config.driver_type == "playwright":
                driver = PlaywrightDriver(headless=True)
            else:
                raise ValueError(f"Unknown driver: {task_config.driver_type}")

            return GeneralScraper(driver, task_config)

        elif isinstance(task_config, RstTaskConfig):
            # TODO: return RstScraper
            pass
        else:
            raise ValueError(f"Unsupported task config type: {type(task_config)}")