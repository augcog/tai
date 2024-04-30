# file_conversion_router/utils/file_utils.py
from pathlib import Path
from typing import Union
from hashlib import sha256


def ensure_path(path: Union[str, Path]) -> Path:
    return Path(path) if not isinstance(path, Path) else path


def calculate_hash(input_path: Path) -> str:
    """Calculate and return the SHA256 hash of the file content."""
    sha256_hash = sha256()
    with open(input_path, "rb") as file:
        for byte_block in iter(lambda: file.read(4096), b""):
            sha256_hash.update(byte_block)
    return sha256_hash.hexdigest()
