from typing import Dict
from dataclasses import dataclass
from pathlib import Path
import os
import json

from rag.file_organizer.src.services.models import LLMBase
from rag.file_organizer.src.services.prompt_service import PromptService
from rag.file_organizer.src.utils.utils import split_into_chunks, find_markdown_files
from rag.file_organizer.src.utils.utils import save_dict_to_json
from rag.file_organizer.src.utils.logging_service import get_logger

logger = get_logger(__name__)


@dataclass
class FileSummary:
    file_path: str
    file_name: str
    summary: str


class Summarizer:
    def __init__(self, config, model: LLMBase):
        self.config = config
        self.model = model
        self.prompt_service = PromptService()

    async def generate_summary(self, file_path: str) -> Dict[str, str]:
        """
        Generate a summary for a given file.

        Args:
            file_path: Path to the file to summarize

        Returns:
            Dictionary with file path as key and summary as value
        """
        # Read file content
        with open(file_path, "r", encoding="utf-8") as f:
            file_content = f.read()

        if len(file_content) < self.config.min_chunk_size:
            logger.info(f"File {file_path} is too short to summarize.")
            return {file_path: "File content is too short to summarize."}

        # Split file content into chunks
        chunks = split_into_chunks(file_content)
        chunk_summaries = []
        for i, chunk in enumerate(chunks, 1):
            # Generate prompt
            messages = self.prompt_service.create_chunk_summary_prompt(chunk=chunk)
            chunk_summary = await self.model.chat(messages=messages)
            chunk_summaries.append(chunk_summary.strip())

        # messages = self.prompt_service.create_combined_summary_prompt(
        #     file_name=Path(file_path).name,
        #     summaries=chunk_summaries
        # )

        messages = self.prompt_service.create_summary_prompt(file_content)

        # Generate summary using LLM
        summary = await self.model.chat(
            messages=messages,
        )

        return {file_path: summary.strip()}

    async def generate_all_summaries(self, file_paths: list) -> Dict[str, str]:
        """
        Generate summaries for multiple files in batch.

        Args:
            file_paths: List of file paths to summarize

        Returns:
            Dictionary mapping file paths to their summaries
        """
        summaries = {}
        for file_path in file_paths:
            try:
                summary = await self.generate_summary(file_path)
                summaries.update(summary)
                logger.info(f"Generated summary for {file_path}")
            except Exception as e:
                summaries[file_path] = f"Error: {str(e)}"
                logger.info(f"Error generating summary for {file_path}: {e}")
                continue
        # sort summaries by file name
        summaries = dict(sorted(summaries.items(), key=lambda item: item[0]))
        return summaries

    async def process_folder(
        self,
        input_folder_path: str | Path,
        output_path: str | Path,
        text_filter: str = None,
    ) -> Dict[str, str]:
        """
        Process all markdown files in a folder and save their summaries.

        Args:
            input_folder_path: Path to the folder containing markdown files
            output_path: Path where to save the summaries JSON file
            text_filter: Optional text to filter filenames

        Returns:
            Dictionary mapping file paths to their summaries

        Raises:
            ValueError: If the input folder doesn't exist
            IOError: If there's an error writing the output file
        """
        # Find all markdown files
        file_paths = find_markdown_files(input_folder_path, text_filter)

        if not file_paths:
            logger.info(f"No markdown files found in {input_folder_path}")
            return {}

        logger.info(f"Found {len(file_paths)} markdown files")

        # Generate summaries for all files
        summaries = await self.generate_all_summaries(file_paths)

        # Save summaries to JSON
        save_dict_to_json(summaries, output_path)
        logger.info(f"Summaries saved to {output_path}")

        return summaries
