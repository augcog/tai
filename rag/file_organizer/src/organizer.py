# organizer/core.py
from __future__ import annotations
from pathlib import Path
import logging
import os

import yaml

from rag.file_organizer.src.config.config import OrganizerCfg
from rag.file_organizer.src.services.models import get_llm
from rag.file_organizer.src.core.summarizer import Summarizer
from rag.file_organizer.src.core.topic_classifier import TopicClassifier
from rag.file_organizer.src.core.func_classifier import FuncClassifier
from rag.file_organizer.src.core.file_organizer import organize_files
from rag.file_organizer.src.utils.logging_service import get_logger
from dotenv import load_dotenv

log = get_logger(__name__)


class FileOrganizer:
    """
    High-level façade that wires together all sub-components.

    Typical use:

        cfg  = OrganizerCfg.from_yaml("organizer.yaml")
        org  = FileOrganizer(cfg)
        org.organize_course("/data/CS61B")
    """

    # -------- construction --------------------------------------------------

    def __init__(self, course_config_path: str | Path):

        load_dotenv()
        course_config_path = Path(course_config_path)
        config_path = Path(__file__).parent.parent / "config.yaml"
        if not config_path.exists():
            raise FileNotFoundError(f"Organizer config file not found: {config_path}")
        self.cfg = OrganizerCfg.from_yaml(config_path)

        if not course_config_path.exists():
            raise FileNotFoundError(f"Course config file not found: {course_config_path}")

        with open(course_config_path, "r") as f:
            course_config = yaml.safe_load(f)
        self.topics = course_config["topic_summaries"]
        self.predefined_classification = course_config.get("predefined_classification", {})

        # Instantiate each component ONCE.  If they're heavy, do it lazily.
        self._summariser = Summarizer(
            config=self.cfg.summarizer,
            model=get_llm(self.cfg.summarizer.llm_profile),
        )
        self._topic_classifier = TopicClassifier(
            config=self.cfg.topic_classifier,
            model=get_llm(self.cfg.topic_classifier.llm_profile)
        )
        self._func_classifier = FuncClassifier(
            config=self.cfg.func_classifier,
            model=get_llm(self.cfg.func_classifier.llm_profile)
        )

    # -------- public high-level API -----------------------------------------
    async def organize_course(
        self,
        course_path: str | Path,
        output_dir: str | Path,
        course_cfg: str | Path | None = None,
    ) -> None:
        """
        End-to-end one-liner for callers.
        """
        course_path = Path(course_path)
        if not course_path.is_dir():
            raise FileNotFoundError(course_path)
        summary_path = str(course_path / "summaries.json")
        topics_classification_path = str(course_path / "topics.json")
        func_classification_path = str(course_path / "functions.json")

        log.info("📂 Organising %s …", course_path.name)
        # Summaries
        await self._summariser.process_folder(course_path, summary_path)
        log.info("✅ Summarized %s", course_path.name)
        # Topic classification
        await self._topic_classifier.process_summaries(summary_path, topics_classification_path, self.topics)
        log.info("✅ Classified topics in %s", course_path.name)
        # Function classification
        await self._func_classifier.process_summaries(summary_path, func_classification_path, self.predefined_classification)
        log.info("✅ Classified functions in %s", course_path.name)
        # reorganize folder
        organize_files(course_path, topics_classification_path, func_classification_path, output_dir, move_files=False)
        log.info("✅ Organised files in %s", course_path.name)

        log.info("✅ Finished %s", course_path.name)

    # -------- public low-level hooks (optional) -----------------------------

    async def summarize_only(
        self,
        input_dir: str | Path,
        output_path: str | Path,
    ) -> None:
        await self._summariser.process_folder(input_dir, output_path)

    async def func_classify_only(
        self,
        input_dir: str | Path,
        output_path: str | Path,
    ) -> None:
        await self._func_classifier.process_summaries(input_dir, output_path, self.predefined_classification)

    async def topic_classify_only(
        self,
        input_dir: str | Path,
        output_path: str | Path,
    ) -> None:
        await self._topic_classifier.process_summaries(input_dir, output_path, self.topics)


if __name__ == "__main__":
    import asyncio

    async def summarize():
        org = FileOrganizer("/Users/lihaichao/Documents/TAI/file_organizer/course_config_sample.yaml")
        await org.summarize_only("/Users/lihaichao/Documents/TAI/file_organizer/tests/CS61A_md/textbook/root", "summaries.json")

    async def topic_classify_summary():
        org = FileOrganizer("/Users/lihaichao/Documents/TAI/file_organizer/course_config_sample.yaml")
        await org.topic_classify_only("/Users/lihaichao/Documents/TAI/file_organizer/src/summaries.json", "topics2.json")

    async def func_classify_summary():
        org = FileOrganizer("/Users/lihaichao/Documents/TAI/file_organizer/course_config_sample.yaml")
        await org.func_classify_only("/Users/lihaichao/Documents/TAI/file_organizer/src/summaries.json", "functions_mock.json")

    async def main():
        org = FileOrganizer("/Users/lihaichao/Documents/TAI/file_organizer/course_config_sample.yaml")
        await org.organize_course("/Users/lihaichao/Documents/TAI/file_organizer/tests/CS61A_md",
                                  "/Users/lihaichao/Documents/TAI/file_organizer/tests/organized_output")

    asyncio.run(main())