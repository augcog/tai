# organizer/utils/logging_service.py
"""
A tiny, process-wide logging helper.

Usage
-----
>>> from src.utils.logging_service import get_logger
>>> logger = get_logger(__name__)
>>> logger.info("Hello from %s", __name__)
"""

from __future__ import annotations
import logging
import logging.handlers
import os
from pathlib import Path
from functools import lru_cache

DEFAULT_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()      # e.g. DEBUG in dev
MAX_BYTES = int(os.getenv("LOG_MAX_BYTES", 5 * 1024 * 1024))  # 5 MB
BACKUP_COUNT = int(os.getenv("LOG_BACKUP_COUNT", 3))       # keep 3 old logs


@lru_cache(maxsize=1)
def _configure_root() -> None:
    """Runs exactly once per process (thanks to lru_cache)."""
    root = logging.getLogger()
    if root.handlers:            # someone else configured logging – respect it
        return

    root.setLevel(DEFAULT_LEVEL)

    # ---- console handler ----
    console_fmt = "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
    console_hdl = logging.StreamHandler()
    console_hdl.setFormatter(logging.Formatter(console_fmt, "%Y-%m-%d %H:%M:%S"))
    root.addHandler(console_hdl)


def get_logger(name: str | None = None) -> logging.Logger:
    """
    Return a logger that follows the global config.
    Call this in every module—inexpensive and thread-safe.

    Parameters
    ----------
    name : str | None
        Defaults to the caller’s `__name__`.  You can pass a custom
        hierarchical name if you want (e.g. "organizer.classifier").
    """
    _configure_root()
    return logging.getLogger(name or __name__)