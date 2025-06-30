import logging
from pathlib import Path
from typing import Dict, List, Optional, Set

import yaml

from scraper.Scraper_master.drivers.playwright_driver import PlaywrightDriver
from scraper.Scraper_master.drivers.requests_driver import RequestsDriver
from scraper.Scraper_master.logger import logger, set_up_logger
from scraper.Scraper_master.scrapers.general_scraper import GeneralScraper
from scraper.Scraper_master.scrapers.scrape_vid import VideoScraper

SCRAPER_MAPPING = {"general_scraper": GeneralScraper, "video_scraper": VideoScraper}

DRIVER_MAPPING = {"request": RequestsDriver, "playwright": PlaywrightDriver}


class WebScraper:
    """Main scraper class that orchestrates the process"""

    def __init__(self, config_path: str):
        # Load configuration
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

        # Create output directories
        self.root_folder = Path(self.config["root_folder"]).resolve()  # absolute path
        self.log_folder = Path(self.config["log_folder"]).resolve()  # absolute path
        self.root_folder.mkdir(exist_ok=True)
        self.log_folder.mkdir(exist_ok=True)
        self.current_depths = {}  # root_url -> current depth
        self.logger = logger
        self.task_folder_path = None

    def run(self):
        """Run all tasks in the configuration"""
        for task in self.config["tasks"]:
            self.process_task(task)

    def process_task(self, task: Dict):
        """Process a single scraping task using DFS"""
        set_up_logger(self.logger, self.log_folder / Path(task["name"] + ".log"))
        self.logger.info(f"Starting task: {task['name']}")

        # Create driver based on task configuration
        if task["driver_type"] not in DRIVER_MAPPING:
            raise ValueError(f"Unknown driver type: {task['driver_type']}")
        driver = DRIVER_MAPPING[task["driver_type"]]()

        self.task_folder_path = self.root_folder / task["name"]

        try:
            # Define root domains for this task
            root_configs = {root["url"]: root for root in task["roots"]}

            # initialize current depths
            self.current_depths = {root_url: 0 for root_url in root_configs.keys()}
            self.current_depths[-61] = -1  # for the starting URL

            # Track visited URLs
            visited = set()

            # Start DFS from the initial URL
            self.dfs_crawl(task["url"], -61, visited, root_configs, driver)

        finally:
            driver.close()
            self.logger.info(f"Completed task: {task['name']}")

    def dfs_crawl(self, url, current_root, visited, root_configs, driver):
        """Recursively crawl URLs using DFS approach"""
        # Skip if already visited
        if url in visited:
            return

        visited.add(url)

        # Determine which root this URL belongs to
        matching_root = None
        for root_url in root_configs:
            if root_url in url:
                matching_root = root_url
                break

        # If no matching root and not the starting URL, skip
        if not matching_root and current_root != -61:
            return

        indent = "         " * self.current_depths[current_root]

        # Handle starting URL or URLs matching a root
        if matching_root and current_root != -61:
            root_config = root_configs[matching_root]
            max_depth = root_config["depth"]
            scraper_type = root_config["scraper_type"]

            # Skip if we've reached max depth for this root
            if self.current_depths[current_root] > max_depth:
                return

            # Get appropriate scraper
            if scraper_type not in SCRAPER_MAPPING:
                raise ValueError(f"Unknown scraper type: {scraper_type}")
            scraper = SCRAPER_MAPPING.get(scraper_type)()

            self.logger.info(f"")
            self.logger.info(f"{indent}Processing: {url}")
            self.logger.info(
                f"{indent}(depth: {self.current_depths[current_root]}, root: {current_root})"
            )
            # Scrape the page
            try:
                links = scraper.scrape(url, driver, self.task_folder_path)
            except Exception as e:
                self.logger.error(f"{indent}Error processing link {url}: {e}")
                return

            # Process each link with DFS
            for link in links:
                self.current_depths[current_root] += 1
                self.dfs_crawl(link, matching_root, visited, root_configs, driver)
                self.current_depths[current_root] -= 1
        else:
            # This is the starting URL, just scrape it using general scraper
            scraper = GeneralScraper()
            self.logger.info(f"Processing starting URL: {url}")
            try:
                links = scraper.scrape(url, driver, self.task_folder_path)
            except Exception as e:
                self.logger.error(f"{indent}Error processing link {url}: {e}")
                return

            # Process each link, checking for matching roots
            for link in links:
                # Determine root for each link
                link_root = None
                for root_url in root_configs:
                    if root_url in link:
                        link_root = root_url
                        break

                if link_root:
                    self.current_depths[link_root] += 1
                    self.dfs_crawl(link, link_root, visited, root_configs, driver)
                    self.current_depths[link_root] -= 1


if __name__ == "__main__":
    scraper = WebScraper("../task.yaml")
    scraper.run()
