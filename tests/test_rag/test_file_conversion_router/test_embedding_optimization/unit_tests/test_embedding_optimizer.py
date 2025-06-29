from unittest.mock import patch

import pytest
import yaml

from rag.file_conversion_router.classes.chunk import Chunk
from rag.file_conversion_router.embedding_optimization.src.pipeline.optimizer import (
    EmbeddingOptimizer,
)


def create_config_file(
    tmp_path,
    tasks,
    model_name="mock-model",
    model_type="test_mock",
    model_path="/path/to/mock-model",
    endpoint=None,
    api_key=None,
):
    """Helper function to create a temporary config file."""
    model_option = {"name": model_name, "type": model_type}
    if model_type == "local":
        model_option["path"] = model_path
    elif model_type == "server":
        model_option["endpoint"] = endpoint
        model_option["api_key"] = api_key

    config = {
        "processing": {"enable": True, "tasks": tasks},
        "models": {"default": model_name, "options": [model_option]},
    }
    config_path = tmp_path / "config.yaml"
    with open(config_path, "w") as f:
        yaml.dump(config, f)
    return config_path


class TestEmbeddingOptimizer:
    """Tests for EmbeddingOptimizer."""

    def test_error_handling(self, config_path_for_test, mock_server_model):
        """Test error handling with invalid chunks."""
        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=mock_server_model,
        ):
            optimizer = EmbeddingOptimizer(config_path_for_test)

            invalid_chunks = [
                Chunk(content="", metadata={"id": 1}),
                Chunk(content="Valid content", metadata={"id": 2}),
            ]

            # Process with fail_fast=False
            processed_chunks = optimizer.process_chunks(invalid_chunks, fail_fast=False)

            # Verify results
            assert len(processed_chunks) == 2

            # Verify first chunk (empty content)
            assert processed_chunks[0].metadata.get("processing_status") == "error"
            assert "error_message" in processed_chunks[0].metadata
            assert (
                processed_chunks[0].metadata["error_message"]
                == "Empty content is not allowed"
            )

            # Verify second chunk (valid content)
            assert processed_chunks[1].metadata.get("processing_status") == "success"

            # Test fail_fast=True
            with pytest.raises(Exception):
                optimizer.process_chunks(invalid_chunks, fail_fast=True)

    def test_metadata_preservation(
        self, config_path_for_test, mock_server_model, sample_chunks
    ):
        """Test that original metadata is preserved and enhanced."""
        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=mock_server_model,
        ):
            optimizer = EmbeddingOptimizer(config_path_for_test)

            # Add some metadata
            for chunk in sample_chunks:
                chunk.metadata["original_length"] = len(chunk.content)

            processed_chunks = optimizer.process_chunks(sample_chunks)

            # Verify metadata
            for original, processed in zip(sample_chunks, processed_chunks):
                assert processed.metadata.get("processing_status") == "success"
                assert (
                    processed.metadata.get("original_length")
                    == original.metadata["original_length"]
                )
                assert "task_results" in processed.metadata

    def test_basic_processing(
        self, config_path_for_test, mock_server_model, sample_chunks
    ):
        """Test basic chunk processing."""
        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=mock_server_model,
        ):
            optimizer = EmbeddingOptimizer(config_path_for_test)
            processed_chunks = optimizer.process_chunks(sample_chunks)

            assert len(processed_chunks) == len(sample_chunks)
            for chunk in processed_chunks:
                assert isinstance(chunk, Chunk)
                assert chunk.content != ""
                assert "task_results" in chunk.metadata
                assert chunk.metadata.get("processing_status") == "success"

    @pytest.mark.cleanup
    def test_cleanup(self, config_path_for_test):
        """Clean up test config file."""
        import os

        if os.path.exists(config_path_for_test):
            os.unlink(config_path_for_test)
