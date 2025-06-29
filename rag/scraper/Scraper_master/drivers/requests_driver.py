import requests
from .driver import Driver, Resp


class RequestsDriver(Driver):
    def __init__(self):
        self.session = requests.Session()

    def download_raw(self, filename, url: str):
        response = requests.get(url, stream=True)
        response.raise_for_status()  # Raise an exception for HTTP errors

        content_type = response.headers.get("Content-Type", "").lower()

        if "text/html" in content_type:
            with open(f'{filename.split(".")[0]}.html', "w", encoding="utf-8") as file:
                file.write(response.text)
        else:
            with open(filename, "wb") as file:
                for chunk in response.iter_content(chunk_size=8192):
                    file.write(chunk)

        return Resp(
            html_content=response.text if "text/html" in content_type else None,
            is_html="text/html" in content_type,
            true_url=response.url,
        )

    def close(self):
        self.session.close()
