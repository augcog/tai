from typing import Dict
from pathlib import Path
import random

from rag.file_organizer.src.utils.utils import *
from rag.file_organizer.src.services.models import LLMBase


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
