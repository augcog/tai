import logging
import os
import tempfile
from pathlib import Path

import pytest
import yaml

from rag.file_conversion_router.classes.chunk import Chunk
from rag.file_conversion_router.embedding_optimization.src.models.mock_model import (
    MockModel,
)
from rag.file_conversion_router.embedding_optimization.src.pipeline.optimizer import (
    EmbeddingOptimizer,
)
from rag.file_conversion_router.embedding_optimization.src.utils import (
    ensure_model_downloaded,
)

# Configure logging for the test
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@pytest.mark.integration
@pytest.mark.skip(
    reason="1. This test takes really long time (2 min for Mac M1) to run for most personal devices.\n"
    "2. Local model needs some additional set up, such as get access from huggingface.\n"
    "Run it only when necessary."
)
def test_embedding_optimizer_integration_with_local_llama_3_2_3b(sample_chunks):
    """
    Integration test for EmbeddingOptimizer to ensure it processes chunks correctly using the LocalLLama3Model.
    This test requires the actual model to be downloaded and available on the local device.
    """
    # Define the model name and directory
    test_path = Path(__file__).parent
    model_name = "meta-llama/Llama-3.2-3B"  # Replace with actual Llama 3 model name
    model_cache_dir = str(
        Path(test_path) / "models" / "Meta-Llama-3.2-3B"
    )  # Replace with actual model directory

    # Ensure the model is downloaded
    try:
        model_cache_path = ensure_model_downloaded(model_name, model_cache_dir)
    except RuntimeError as e:
        pytest.skip(f"Model download failed: {e}")

    # Create a temporary config file
    config = {
        "processing": {"enable": True, "tasks": ["summarize"]},
        "models": {
            "default": model_name,
            "options": [
                {"name": model_name, "type": "local", "path": model_cache_path}
            ],
        },
    }

    # Write config to a temporary file
    with tempfile.NamedTemporaryFile(
        mode="w+", delete=False, suffix=".yaml"
    ) as tmp_config:
        yaml.dump(config, tmp_config)
        config_path = tmp_config.name

    # Initialize EmbeddingOptimizer
    optimizer = EmbeddingOptimizer(config_path=config_path)

    # Process chunks
    # Copy the old content for comparison
    copy_old_chunk_content = [chunk.content for chunk in sample_chunks]
    optimized_chunks = optimizer.process_chunks(sample_chunks)

    # Assertions
    assert len(optimized_chunks) == 2, "Should process two chunks."

    assert len(optimized_chunks[0].content) > 0, "Content should not be empty."
    assert "TAI" in optimized_chunks[0].content, "Content should contain 'TAI'."
    assert (
        optimized_chunks[0].content != copy_old_chunk_content[0]
    ), "Content should be different."
    # assert len(optimized_chunks[0].content) < len(copy_old_chunk_content[0]), "Content should be shorter."

    assert len(optimized_chunks[1].content) > 0, "Content should not be empty."
    assert (
        "transfer learning" in optimized_chunks[1].content
    ), "Content should contain 'TAI'."
    assert (
        optimized_chunks[1].content != copy_old_chunk_content[1]
    ), "Content should be different."
    # assert len(optimized_chunks[1].content) < len(copy_old_chunk_content[1]), "Content should be shorter."


@pytest.mark.integration
def test_server_model_generate_summary_integration(
    server_model_tai, config_path_for_test
):
    """Integration test for EmbeddingOptimizer to ensure it correctly processes summaries using the server model.

    Prerequisites:
        - The backend server must be running and accessible at the specified endpoint.
        - If authentication is required, ensure that the API key is valid.
    """
    optimizer = EmbeddingOptimizer(config_path_for_test)

    sample_chunks = [
        Chunk(
            content="Someone told Perry that Artificial intelligence (AI) is intelligence demonstrated by machines, unlike the natural intelligence displayed by humans and animals.",
            metadata={"test_id": 1},
        ),
        Chunk(
            content="Perry learned from TAI that Machine learning (ML) is a subset of AI that involves the study of computer algorithms that improve automatically through experience.",
            metadata={"test_id": 2},
        ),
    ]

    try:
        processed_chunks = optimizer.process_chunks(sample_chunks)

        assert len(processed_chunks) == len(
            sample_chunks
        ), "Number of processed chunks should match input"

        for idx, chunk in enumerate(processed_chunks):
            logger.info(f"Original text {idx + 1}: {sample_chunks[idx].content}")
            logger.info(f"Generated summary: {chunk.content}")

            assert isinstance(chunk.content, str), "Summary should be a string"
            assert len(chunk.content) > 0, "Summary should not be empty"
            assert (
                chunk.content != sample_chunks[idx].content
            ), "Summary should differ from original text"

            assert (
                chunk.metadata["test_id"] == sample_chunks[idx].metadata["test_id"]
            ), "Metadata should be preserved"
            assert (
                "task_results" in chunk.metadata
            ), "Task results should be included in metadata"

    except Exception as e:
        pytest.fail(f"EmbeddingOptimizer processing raised an exception: {e}")
    finally:
        # Cleanup the temporary config file
        os.unlink(config_path_for_test)


def test_server_model_handle_markdown(config_path_for_test, server_model_tai):
    """Test that processed markdown maintains proper formatting."""
    optimizer = EmbeddingOptimizer(config_path_for_test)

    markdown_content = """
    # Test Header

    ## Subheader

    1. Numbered item
    2. Another item

    - Bullet point
    - Another point

    [Link](http://example.com)
    """

    result = optimizer.process_markdown(markdown_content)

    assert result.success
    processed_content = result.content

    # Verify basic markdown elements are preserved
    assert "#" in processed_content  # Headers
    assert "*" in processed_content or "-" in processed_content  # Lists


class TestEmbeddingOptimizerWithMockModel:
    """Tests for EmbeddingOptimizer using MockModel."""

    def test_basic_processing(self, mock_config_path, sample_chunks):
        """Test basic chunk processing with MockModel."""
        # Initialize optimizer
        optimizer = EmbeddingOptimizer(mock_config_path)

        # Verify that we got a MockModel
        assert isinstance(optimizer.model, MockModel)

        # Process chunks
        processed_chunks = optimizer.process_chunks(sample_chunks)

        # Verify basic processing
        assert len(processed_chunks) == len(sample_chunks)
        for chunk in processed_chunks:
            assert isinstance(chunk, Chunk)
            assert chunk.content != ""
            assert "task_results" in chunk.metadata
            assert "processing_status" in chunk.metadata
            assert chunk.metadata["processing_status"] == "success"

    def test_task_execution_flow(self, mock_config_path, sample_chunks):
        """Test that tasks are executed in the correct order."""
        # Initialize optimizer with tracking enabled
        optimizer = EmbeddingOptimizer(mock_config_path)
        mock_model = optimizer.model

        # Process chunks
        processed_chunks = optimizer.process_chunks(sample_chunks)

        # Get call history
        history = mock_model.get_call_history()

        # Verify task execution order
        assert len(history) > 0
        # First calls should be for 'summarize' task
        assert "summarize" in history[0]["prompt"].lower()
        # Later calls should include 'enhance' task
        assert any("enhance" in call["prompt"].lower() for call in history)
        # Final calls should include 'combine' text
        assert any("combine" in call["prompt"].lower() for call in history)

    def test_error_handling(self, mock_config_path, sample_chunks):
        """Test error handling with failing MockModel."""
        # Initialize optimizer with failing model
        optimizer = EmbeddingOptimizer(mock_config_path)
        optimizer.model.set_should_fail(True)

        # Process with fail_fast=False
        processed_chunks = optimizer.process_chunks(sample_chunks, fail_fast=False)
        assert len(processed_chunks) == len(sample_chunks)
        for chunk in processed_chunks:
            assert chunk.metadata["processing_status"] == "error"
            assert "error_message" in chunk.metadata

        # Process with fail_fast=True
        optimizer.model.clear_call_history()
        with pytest.raises(Exception):
            optimizer.process_chunks(sample_chunks, fail_fast=True)

    def test_metadata_preservation(self, mock_config_path, sample_chunks):
        """Test that original metadata is preserved and enhanced."""
        optimizer = EmbeddingOptimizer(mock_config_path)

        # Add some metadata
        for chunk in sample_chunks:
            chunk.metadata["original_length"] = len(chunk.content)

        processed_chunks = optimizer.process_chunks(sample_chunks)

        # Verify metadata
        for original, processed in zip(sample_chunks, processed_chunks):
            assert (
                processed.metadata["original_length"]
                == original.metadata["original_length"]
            )
            assert "task_results" in processed.metadata

    def test_mock_response_customization(self, mock_config_path, sample_chunks):
        """Test with customized mock responses."""
        optimizer = EmbeddingOptimizer(mock_config_path)

        # Set custom response
        optimizer.model.set_response("Custom mock response")

        processed_chunks = optimizer.process_chunks(sample_chunks)

        # Verify custom response
        for chunk in processed_chunks:
            assert "Custom mock response" in chunk.content

    def test_performance_simulation(self, mock_config_path, sample_chunks):
        """Test with simulated processing delay."""
        optimizer = EmbeddingOptimizer(mock_config_path)

        # Set small delay for testing
        optimizer.model.set_delay(0.1)

        import time

        start_time = time.time()

        processed_chunks = optimizer.process_chunks(sample_chunks)

        elapsed_time = time.time() - start_time
        assert elapsed_time >= 0.1 * len(processed_chunks)

    @pytest.mark.cleanup
    def test_cleanup(self, mock_config_path):
        """Clean up test config file."""
        if Path(mock_config_path).exists():
            Path(mock_config_path).unlink()
