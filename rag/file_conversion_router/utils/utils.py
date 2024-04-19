# file_conversion_router/utils/file_utils.py
from pathlib import Path
from typing import Union


def ensure_path(path: Union[str, Path]) -> Path:
    return Path(path) if not isinstance(path, Path) else path
