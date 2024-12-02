# file_conversion_router/utils/file_utils.py
from hashlib import sha256
from pathlib import Path
from typing import Union
import requests


def ensure_path(path: Union[str, Path]) -> Path:
    return Path(path) if not isinstance(path, Path) else path


def calculate_hash(input_path: Path) -> str:
    """Calculate and return the SHA256 hash of the file content."""
    sha256_hash = sha256()
    with open(input_path, "rb") as file:
        for byte_block in iter(lambda: file.read(4096), b""):
            sha256_hash.update(byte_block)
    return sha256_hash.hexdigest()


def load_conversion_version(version_file_path):
    with open(version_file_path, "r") as f:
        return f.read().strip()


def is_empty_md(file_hash):
    return file_hash == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"


def check_url(url: str) -> int:
    try:
        response = requests.head(url, allow_redirects=True, timeout=10)
        return response.status_code
    except requests.exceptions.RequestException as e:
        print(f"Error accessing {url}: {e}")
        return None