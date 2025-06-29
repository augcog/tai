from abc import ABC, abstractmethod
from dataclasses import dataclass


class Driver(ABC):
    @abstractmethod
    def download_raw(self, filename, url):
        pass

    @abstractmethod
    def close(self):
        pass


@dataclass
class Resp:
    html_content: str
    is_html: bool
    true_url: str
