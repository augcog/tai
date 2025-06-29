from typing import Dict, List
import json
import random
import ast


from rag.file_organizer.src.services.models import LLMBase
from rag.file_organizer.src.services.prompt_service import PromptService
from rag.file_organizer.src.config.config import TopicClassifierCfg
from rag.file_organizer.src.utils.utils import save_dict_to_json
from rag.file_organizer.src.utils.logging_service import get_logger

logger = get_logger(__name__)


class TopicClassifier:
    def __init__(self, config: TopicClassifierCfg, model: LLMBase):
        self.config = config
        self.model = model  # Initialize LLM service
        self.prompt_service = PromptService

    def _generate_mock_topics(self, file_path: str, topics_list: list) -> List[str]:
        """Generate mock topics for testing purposes."""
        return random.sample(
            topics_list, min(len(topics_list), self.config.max_topic_num)
        )

    async def classify_file(
        self, file_path: str, summary: str, topics_list: Dict[str, str]
    ) -> Dict:
        """
        Classify a file into topics based on its summary.

        Args:
            file_path: Path to the file
            summary: Summary of the file content

        Returns:
            Dictionary containing the file's topics
        """
        message = self.prompt_service.create_classification_prompt(summary, topics_list)
        response = await self.model.chat(messages=message)
        topics = response.strip().split(",") if response else []
        return {file_path: topics}

    async def batch_process_summaries(
        self, summaries: Dict[str, str], topics_list: Dict[str, str]
    ) -> Dict:
        message = self.prompt_service.create_classification_prompt(
            str(summaries), topics_list
        )
        response = await self.model.chat(messages=message)
        return ast.literal_eval(response.strip()) if response else {}

    async def process_summaries(
        self, summaries_path: str, output_path: str, topics_list: Dict[str, str]
    ) -> Dict:
        """Process all summaries and classify them into topics."""
        with open(summaries_path, "r") as f:
            summaries = json.load(f)

        if not summaries:
            logger.error(f"No summaries found in {summaries_path}")
            return {}

        logger.info(f"Found {len(summaries)} summaries to process")

        results = {}
        # use batch processing by chunking summaries into smaller parts
        chunk_size = 10
        chunks = [
            list(summaries.items())[i : i + chunk_size]
            for i in range(0, len(summaries), chunk_size)
        ]
        for chunk in chunks:
            chunk_dict = dict(chunk)
            logger.info(f"Processing chunk with {len(chunk_dict)} summaries")
            chunk_results = await self.batch_process_summaries(chunk_dict, topics_list)
            results.update(chunk_results)

        save_dict_to_json(results, output_path)
        logger.info(f"Classified topics saved to {output_path}")
        return results
