import random
from pathlib import Path
from typing import Dict

from file_organizer.src.services.models import LLMBase
from file_organizer.src.utils.utils import *


class TopicProvider:
    def __init__(self, config, model: LLMBase):
        self.model = model
        self.config = config

    def create_topics(self) -> Dict[str, str]:
        """
        Create topics based on the provided configuration.

        Returns:
            Dictionary with topic names as keys and their descriptions as values.
        """
        pass
