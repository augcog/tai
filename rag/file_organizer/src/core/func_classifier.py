from typing import Dict, List, Set
import json
import os
from pathlib import Path
import ast

from rag.file_organizer.src.services.models import LLMBase
from rag.file_organizer.src.config.config import FuncClassifierCfg
from rag.file_organizer.src.services.prompt_service import PromptService
from rag.file_organizer.src.utils.utils import save_dict_to_json
from rag.file_organizer.src.utils.logging_service import get_logger

logger = get_logger(__name__)


class FuncClassifier:
    def __init__(self, config: FuncClassifierCfg, model: LLMBase):
        self.config = config
        self.model = model
        self.prompt_service = PromptService

    def _get_predefined_function(
        self, file_path: str, predefined_classifications
    ) -> str:
        """Check if file path matches any predefined classification patterns."""
        for func, folders in predefined_classifications.items():
            if any(folder in file_path for folder in folders):
                return func
        return None

    async def batch_classify(self, summaries: Dict[str, str]) -> Dict:
        """Classify a file based on its content."""
        message = self.prompt_service.create_func_classification_prompt(str(summaries))
        response = await self.model.chat(messages=message)
        return ast.literal_eval(response.strip()) if response else {}

    async def process_summaries(
        self,
        summaries_path: str,
        output_path: str,
        predefined_classification: Dict = None,
    ) -> Dict:
        """Process all files in a folder and classify them."""
        results = {}

        with open(summaries_path, "r") as f:
            summaries = json.load(f)

        if not summaries:
            logger.error(f"No summaries found in {summaries_path}")
            return {}
        logger.info(f"Found {len(summaries)} summaries to process")

        for file_path, summary in summaries.items():
            if predefined_classification:
                predefined_func = self._get_predefined_function(
                    file_path, predefined_classification
                )
                if predefined_func:
                    results[file_path] = predefined_func

        rest_summaries = summaries.copy()
        for file_path, summary in summaries.items():
            if file_path in results:
                # delete from summaries
                del rest_summaries[file_path]

        # use batch processing by chunking summaries into smaller parts
        chunk_size = 10
        chunks = [
            list(rest_summaries.items())[i : i + chunk_size]
            for i in range(0, len(rest_summaries), chunk_size)
        ]
        for chunk in chunks:
            chunk_dict = dict(chunk)
            chunk_results = await self.batch_classify(chunk_dict)
            results.update(chunk_results)

        save_dict_to_json(results, output_path)
        logger.info(f"Function classification results saved to {output_path}")
        return results
