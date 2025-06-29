"""Compared to the default NougatConfig, TAINougatConfig includes an additional configurable field, using_torch

Currently, other TAI Nougat functionalities, such as Dependency Injection, are not configurable.
"""

import logging
from dataclasses import dataclass, field
from typing import Callable, Union

from .nougat_config import NougatConfig

from rag.file_conversion_router.utils.hardware_detection import detect_is_apple_silicon

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def determine_using_torch() -> bool:
    is_apple_silicon = detect_is_apple_silicon()
    logger.info(f"Apple Silicon detected: {is_apple_silicon}")
    # By default, using torch is False on Apple Silicon, True otherwise
    return not is_apple_silicon


@dataclass
class TAINougatConfig(NougatConfig):
    # Allow either a boolean or a callable that returns a boolean
    using_torch: Union[bool, Callable[[], bool]] = field(
        default_factory=determine_using_torch
    )

    def __post_init__(self):
        if callable(self.using_torch):
            self.using_torch = self.using_torch()

        if self.using_torch:
            logger.info("Using Torch for Nougat PDF conversion.")
        else:
            logger.info(
                "Using MLX for Nougat PDF conversion for Apple Silicon for better performance."
            )
