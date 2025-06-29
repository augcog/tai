import tempfile
from typing import List
from unittest.mock import patch

import pytest
import yaml

from rag.file_conversion_router.classes.chunk import Chunk
from rag.file_conversion_router.embedding_optimization.src.pipeline.optimizer import (
    EmbeddingOptimizer,
)


class OrderTrackingMock:
    """Mock class to track task execution order."""

    def __init__(self):
        self.execution_order = []

    def generate(self, prompt: str) -> str:
        """Mock generate method that tracks task execution based on prompt content."""
        if "summarize" in prompt.lower():
            self.execution_order.append("summarize")
            return "Mocked summary"
        elif "enhance" in prompt.lower():
            self.execution_order.append("enhance")
            return "Mocked enhancement"
        elif "translate" in prompt.lower():
            self.execution_order.append("translate")
            return "Mocked translation"
        elif "combine" in prompt.lower():
            self.execution_order.append("combine")
            return "Mocked combination"
        return "Default mock response"


@pytest.fixture
def complex_config_path():
    """Create a complex configuration with various task types and dependencies."""
    config = {
        "variables": {
            "max_length": 100,
            "style": "professional",
            "language": "english",
        },
        "tasks": {
            # Basic tasks
            "summarize": {
                "type": "prompt",
                "prompt_template": "Please summarize: $content",
            },
            "enhance": {
                "type": "prompt",
                "prompt_template": "Please enhance: $content",
            },
            "translate": {
                "type": "prompt",
                "prompt_template": "Please translate: $content",
            },
            # Sequential task that runs summarize then enhance
            "sequential_process": {
                "type": "sequential",
                "sequence": ["summarize", "enhance"],
            },
            # Composed task that combines summary and translation
            "parallel_process": {
                "type": "composed",
                "subtasks": ["summarize", "translate"],
                "final_prompt": """
                Combine these results:
                Summary: $result_summarize
                Translation: $result_translate
                """,
            },
            # Complex nested task
            "complex_process": {
                "type": "sequential",
                "sequence": [
                    "parallel_process",  # First run parallel processing
                    "sequential_process",  # Then run sequential processing
                ],
            },
        },
        "pipeline": {
            "chunk_task": "complex_process",
            "markdown_task": "complex_process",
            "batch_size": 1,
        },
        "models": {
            "default": "test_model",
            "options": [
                {
                    "name": "test_model",
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


class TestTaskOrdering:
    """Unit tests for task ordering and execution flow."""

    def test_sequential_task_order(self, complex_config_path):
        """Test that sequential tasks are executed in the correct order."""
        tracker = OrderTrackingMock()

        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=tracker,
        ):
            optimizer = EmbeddingOptimizer(complex_config_path)

            # Override pipeline to test sequential_process directly
            optimizer.config.pipeline_settings.chunk_task = "sequential_process"

            chunk = Chunk(content="Test content", metadata={})
            optimizer.process_chunks([chunk])

            # Verify order: summarize -> enhance
            assert tracker.execution_order == ["summarize", "enhance"]

    def test_composed_task_execution(self, complex_config_path):
        """Test that composed tasks execute subtasks and combine results."""
        tracker = OrderTrackingMock()

        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=tracker,
        ):
            optimizer = EmbeddingOptimizer(complex_config_path)

            # Override pipeline to test parallel_process directly
            optimizer.config.pipeline_settings.chunk_task = "parallel_process"

            chunk = Chunk(content="Test content", metadata={})
            optimizer.process_chunks([chunk])

            # Verify subtasks executed and results combined
            expected_tasks = set(["summarize", "translate", "combine"])
            assert set(tracker.execution_order) == expected_tasks
            assert len(tracker.execution_order) == 3  # Each task executed once

    def test_complex_nested_task_flow(self, complex_config_path):
        """Test complex nested task execution with both sequential and composed tasks."""
        tracker = OrderTrackingMock()

        with patch(
            "rag.file_conversion_router.embedding_optimization.src.pipeline.builder.ServerModelTAI",
            return_value=tracker,
        ):
            optimizer = EmbeddingOptimizer(complex_config_path)

            # Use complex_process as main task
            chunk = Chunk(content="Test content", metadata={})
            result = optimizer.process_chunks([chunk])

            # Verify the complete execution flow
            execution_flow = tracker.execution_order

            # Check that all required tasks were executed
            assert "summarize" in execution_flow
            assert "translate" in execution_flow
            assert "combine" in execution_flow
            assert "enhance" in execution_flow

            # Verify task results in metadata
            task_results = result[0].metadata["task_results"]
            assert "parallel_process" in task_results
            assert "sequential_process" in task_results

    def test_task_dependency_validation(self, complex_config_path):
        """Test that task dependencies are properly validated."""
        # Load and modify config to create invalid dependency
        with open(complex_config_path) as f:
            config = yaml.safe_load(f)

        # Add invalid dependency
        config["tasks"]["invalid_task"] = {
            "type": "sequential",
            "sequence": ["nonexistent_task"],
        }
        config["pipeline"]["main_task"] = "invalid_task"

        # Write modified config
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
            yaml.safe_dump(config, tmp)
            invalid_config_path = tmp.name

        # Verify that invalid dependencies are caught
        with pytest.raises(Exception) as exc_info:
            EmbeddingOptimizer(invalid_config_path)
        assert "not found" in str(exc_info.value)

    def test_task_cycle_detection(self, complex_config_path):
        """Test that circular dependencies are detected."""
        # Load and modify config to create a cycle
        with open(complex_config_path) as f:
            config = yaml.safe_load(f)

        # Create circular dependency
        config["tasks"]["task_a"] = {"type": "sequential", "sequence": ["task_b"]}
        config["tasks"]["task_b"] = {"type": "sequential", "sequence": ["task_a"]}
        config["pipeline"]["main_task"] = "task_a"

        # Write modified config
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
            yaml.safe_dump(config, tmp)
            cycle_config_path = tmp.name

        # Verify that cycle is detected
        with pytest.raises(Exception) as exc_info:
            EmbeddingOptimizer(cycle_config_path)
        assert (
            "cycle" in str(exc_info.value).lower()
            or "circular" in str(exc_info.value).lower()
        )

    def verify_execution_metadata(
        self, chunk: Chunk, expected_tasks: List[str]
    ) -> None:
        """Helper method to verify task execution metadata."""
        assert "task_results" in chunk.metadata
        results = chunk.metadata["task_results"]
        for task in expected_tasks:
            assert task in results
        assert chunk.metadata["processing_status"] == "success"
