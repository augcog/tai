from rag.scraper.Scraper_master.base_scraper import BaseScraper

class ScrapeVid(BaseScraper):
    def __init__(self, url, root_filename):
        super().__init__(url)
        self.root_filename = root_filename

    def get_content(self, url):
        """
        Retrieves content from the given URL
        - url (str): URL to fetch the content from.
        - Returns: The content fetched from the URL.
        """
        # Fetch the content from the URL
        headers = {'Accept': 'application/json'}
        response = requests.get(url, headers=headers)
        data = response.json()
        content = data['payload']['blob']['rawLines']

        # Convert the content list into a single string
        content = '\n'.join(content)

        return content