import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Union

from file_conversion_router.classes.chunk import Chunk
from file_conversion_router.embedding_optimization.src.tasks.task_context import (
    TaskContext,
)

from .builder import PipelineBuilder

logger = logging.getLogger(__name__)


@dataclass
class ProcessingResult:
    """Contains the results of content processing."""

    content: Union[str, Chunk]  # Can be either processed text or chunk
    success: bool
    error: Optional[str] = None
    metadata: Dict[str, Any] = None


class ContentProcessor(ABC):
    """Base class for content processors."""

    @abstractmethod
    def validate(self, content: Any) -> Optional[str]:
        """Validate content before processing."""
        pass

    @abstractmethod
    def process(self, content: Any) -> ProcessingResult:
        """Process the content."""
        pass


class ChunkProcessor(ContentProcessor):
    """Processor for chunks."""

    def __init__(self, task_runner, config):
        self.task_runner = task_runner
        self.config = config

    def validate(self, chunk: Chunk) -> Optional[str]:
        """Validate chunk content."""
        if not chunk.content or not chunk.content.strip():
            return "Empty content is not allowed"
        return None

    def process(self, chunk: Chunk) -> ProcessingResult:
        """Process a single chunk."""
        error = self.validate(chunk)
        if error:
            return ProcessingResult(
                content=Chunk(
                    content=chunk.content,
                    titles=chunk.titles,
                    chunk_url=chunk.chunk_url,
                    metadata={
                        **(chunk.metadata or {}),
                        "processing_status": "error",
                        "error_message": error,
                    },
                ),
                success=False,
                error=error,
            )

        try:
            context = TaskContext(
                chunk=chunk, results={}, variables=self.config.variables
            )

            # Use proper attribute access instead of dictionary access
            processed_content = self.task_runner.execute_task(
                self.config.pipeline_settings.chunk_task,  # Use attribute access
                context,
            )

            return ProcessingResult(
                content=Chunk(
                    content=processed_content,
                    titles=chunk.titles,
                    chunk_url=chunk.chunk_url,
                    metadata={
                        **(chunk.metadata or {}),
                        "task_results": context.results,
                        "processing_status": "success",
                    },
                ),
                success=True,
            )

        except Exception as e:
            error_msg = str(e)
            logger.error(f"Error processing chunk: {error_msg}", exc_info=True)
            return ProcessingResult(
                content=Chunk(
                    content=chunk.content,
                    titles=chunk.titles,
                    chunk_url=chunk.chunk_url,
                    metadata={
                        **(chunk.metadata or {}),
                        "processing_status": "error",
                        "error_message": error_msg,
                    },
                ),
                success=False,
                error=error_msg,
            )


class MarkdownProcessor(ContentProcessor):
    """Processor for markdown content."""

    def __init__(self, task_runner, config):
        self.task_runner = task_runner
        self.config = config

    def validate(self, content: str) -> Optional[str]:
        """Validate markdown content."""
        if not content or not content.strip():
            return "Empty markdown content"
        return None

    def process(self, content: str) -> ProcessingResult:
        """Process markdown content."""
        error = self.validate(content)
        if error:
            return ProcessingResult(
                content=content,
                success=False,
                error=error,
                metadata={"processing_status": "error"},
            )

        try:
            context = TaskContext(
                chunk=Chunk(content=content),
                results={},
                variables=self.config.variables,
            )

            # Use proper attribute access
            processed_content = self.task_runner.execute_task(
                self.config.pipeline_settings.markdown_task,  # Use attribute access
                context,
            )

            return ProcessingResult(
                content=processed_content,
                success=True,
                metadata={
                    "processing_status": "success",
                    "task_results": context.results,
                },
            )

        except Exception as e:
            error_msg = str(e)
            logger.error(f"Error processing markdown: {error_msg}", exc_info=True)
            return ProcessingResult(
                content=content,
                success=False,
                error=error_msg,
                metadata={"processing_status": "error"},
            )


class EmbeddingOptimizer:
    """
    Main optimizer class that provides interfaces for processing both
    markdown content and chunks using LLM.
    """

    def __init__(self, config_path: str):
        """Initialize the optimizer with configuration."""
        self.config, self.model, self.task_runner = PipelineBuilder.build(config_path)

        # Initialize processors
        self.markdown_processor = MarkdownProcessor(self.task_runner, self.config)
        self.chunk_processor = ChunkProcessor(self.task_runner, self.config)

        logger.info(
            f"Initialized EmbeddingOptimizer with configuration from {config_path}"
        )

    def process_markdown(self, content: str) -> ProcessingResult:
        """
        Process markdown content.

        Args:
            content: Markdown text to process

        Returns:
            ProcessingResult containing processed content
        """
        logger.info("Processing markdown content")
        return self.markdown_processor.process(content)

    def process_chunks(
        self, chunks: List[Chunk], fail_fast: bool = False
    ) -> List[Chunk]:
        """
        Process a list of chunks.

        Args:
            chunks: List of chunks to process
            fail_fast: If True, raise exception on first error

        Returns:
            List of processed chunks

        Raises:
            Exception: If fail_fast is True and processing fails
        """
        logger.info(f"Processing {len(chunks)} chunks")
        results = []

        for i, chunk in enumerate(chunks):
            logger.debug(f"Processing chunk {i + 1}/{len(chunks)}")
            result = self.chunk_processor.process(chunk)

            if not result.success and fail_fast:
                raise Exception(f"Failed to process chunk {i + 1}: {result.error}")

            results.append(result.content)

        successful = sum(
            1 for r in results if r.metadata.get("processing_status") == "success"
        )
        logger.info(
            f"Completed processing: {successful}/{len(chunks)} chunks successful"
        )

        return results
