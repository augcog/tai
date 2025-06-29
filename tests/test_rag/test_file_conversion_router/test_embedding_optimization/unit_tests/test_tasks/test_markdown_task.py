import logging
import tempfile
from unittest.mock import patch

import pytest
import yaml

from rag.file_conversion_router.embedding_optimization.src.pipeline.optimizer import (
    EmbeddingOptimizer,
)

logger = logging.getLogger(__name__)


@pytest.fixture
def markdown_config():
    """Create configuration with markdown processing tasks."""
    config = {
        "variables": {"max_length": 100, "style": "professional", "format": "markdown"},
        "tasks": {
            "process_markdown": {
                "type": "prompt",
                "prompt_template": """
                Process this markdown content maintaining proper formatting:

                $content

                Style: $style
                Format: $format
                """,
            },
            "enhance_chunk": {
                "type": "prompt",
                "prompt_template": """
                Enhance this chunk:
                $content
                """,
            },
        },
        "pipeline": {
            "markdown_task": "process_markdown",
            "chunk_task": "enhance_chunk",
            "batch_size": 2,
        },
        "models": {
            "default": "server_mock",
            "options": [
                {
                    "name": "server_mock",
                    "type": "server",
                    "endpoint": "mock_endpoint",
                    "api_key": "mock_key",
                }
            ],
        },
    }

    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
        yaml.safe_dump(config, tmp)
        return tmp.name


class TestMarkdownProcessing:
    """Tests for markdown processing functionality."""

    def test_basic_markdown_processing(self, markdown_config, mock_server_model):
        """Test basic markdown content processing."""
        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=mock_server_model,
        ):
            optimizer = EmbeddingOptimizer(markdown_config)

            markdown_content = """
            # Test Header

            This is a test paragraph with **bold** and *italic* text.

            - List item 1
            - List item 2
            """

            result = optimizer.process_markdown(markdown_content)

            assert result.success
            assert isinstance(result.content, str)
            assert result.metadata["processing_status"] == "success"
            assert "task_results" in result.metadata
            assert result.content == "Default mocked response"

    def test_empty_markdown_handling(self, markdown_config, mock_server_model):
        """Test handling of empty markdown content."""
        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=mock_server_model,
        ):
            optimizer = EmbeddingOptimizer(markdown_config)

            # Test with empty string
            result = optimizer.process_markdown("")
            assert not result.success
            assert result.metadata["processing_status"] == "error"
            assert "Empty markdown content" in result.error

            # Test with whitespace
            result = optimizer.process_markdown("   \n   ")
            assert not result.success
            assert result.metadata["processing_status"] == "error"
            assert "Empty markdown content" in result.error

    def test_markdown_with_special_characters(self, markdown_config, mock_server_model):
        """Test processing markdown with special characters."""
        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=mock_server_model,
        ):
            optimizer = EmbeddingOptimizer(markdown_config)

            markdown_content = """
            # Special Characters Test

            Code block:
            ```python
            def test():
                print("Hello!")
            ```

            Table:
            | Col1 | Col2 |
            |------|------|
            | Data | More |

            Symbols: & * ^ % $ # @
            """

            result = optimizer.process_markdown(markdown_content)

            assert result.success
            assert result.metadata["processing_status"] == "success"
            assert isinstance(result.content, str)

    def test_large_markdown_processing(self, markdown_config, mock_server_model):
        """Test processing large markdown content."""
        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=mock_server_model,
        ):
            optimizer = EmbeddingOptimizer(markdown_config)

            # Generate large markdown content
            large_content = "\n".join(
                [
                    f"# Section {i}\n\nThis is content for section {i}."
                    for i in range(50)
                ]
            )

            result = optimizer.process_markdown(large_content)

            assert result.success
            assert result.metadata["processing_status"] == "success"
            assert isinstance(result.content, str)

    @pytest.mark.cleanup
    def test_cleanup(self, markdown_config):
        """Clean up test config file."""
        import os

        if os.path.exists(markdown_config):
            os.unlink(markdown_config)
