class Page:
    def __init__(self, content: dict, filetype: str, page_url: str = ""):
        # dictionary of attributes
        self.content = content
        # file type (md, pdf, etc.)
        self.filetype = filetype
        # page url
        self.page_url = page_url

    def to_file(self, output_path: str) -> None:
        """Write the page content to a file."""
        with open(output_path, "w") as f:
            # f.write(self.content)
            f.write(self.content)

