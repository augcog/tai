import pytest

from file_conversion_router.embedding_optimization.src.models.local_model import (
    LocalLLama3Model,
)
from file_conversion_router.embedding_optimization.src.models.server_model_tai import (
    ServerModelTAI,
)


@pytest.fixture
def sample_chunks():
    """Fixture for sample chunks used in tests."""
    chunk1 = Chunk(
        titles="Title 1",
        content="""This is a long content that needs to be summarized.
User: "What is TAI?"
Assistant: "TAI is an open source project developed by researchers and students at UC Berkeley (see Credits below), with the goal to offer the power of edge GPT models and services for education purposes."
        """,
        chunk_url=["https://github.com/augcog/tai"],
    )
    chunk2 = Chunk(
        titles="Title 2",
        content="""Another piece of content that requires summarization.
User: "Can you explain how transfer learning works in neural networks, particularly in image classification?"

Assistant: "Certainly! Transfer learning is a technique where a model trained on a large dataset, like ImageNet, is fine-tuned on a smaller, specific dataset. The model’s early layers, which capture general features like edges and shapes, are retained, while later layers are adjusted for the new task."

User: "So, it helps reuse learned features for new tasks without starting from scratch?"

Assistant: "Exactly! It’s efficient because the model leverages pre-learned features, reducing training time and improving performance, especially when labeled data is limited."
        """,
        chunk_url=["https://example.com/2"],
    )
    return [chunk1, chunk2]


@pytest.fixture
def mock_model():
    """Fixture for a mock model used in testing."""
    return MockModel()


@pytest.fixture
def local_llama3_model():
    """Fixture for the local Llama 3 model."""
    return LocalLLama3Model(
        model_path="/path/to/model/Meta-Llama-3.2-3B",
        model_name="meta-llama/Llama-3.2-3B",
    )


@pytest.fixture
def server_model_tai():
    """Fixture for the server model used in testing."""
    test_endpoint = "https://tai.berkeley.edu/api/chat"  # Update if different

    user_id = "test_user@example.com"
    course_id = "default"

    # Optional: Define an API key if your backend requires authentication
    api_key = "your_test_api_key_here"  # Replace with a valid API key or set to None if not needed

    # Initialize the ServerModel
    server_model_tai = ServerModelTAI(
        endpoint=test_endpoint,
        user_id=user_id,
        course_id=course_id,
        api_key=api_key,  # Set to None if authentication is not required
        timeout=30,
    )
    return server_model_tai


@pytest.fixture
def config_path_for_test():
    """Fixture providing test configuration file."""
    config = {
        "variables": {"max_length": 100, "style": "concise", "language": "english"},
        "tasks": {
            "summarize": {
                "type": "prompt",
                "prompt_template": """
                Please provide a concise summary of the following text:

                $content

                Maximum length: $max_length words
                Style: $style
                Language: $language
                """,
            },
            "enhance": {
                "type": "prompt",
                "prompt_template": """
                Enhance the following text to be more professional:

                $content
                """,
            },
            "combined_processing": {
                "type": "composed",
                "subtasks": ["summarize", "enhance"],
                "final_prompt": """
                Combine these results into a final version:

                Summary: $result_summarize
                Enhanced: $result_enhance

                Create a final version that combines both while maintaining clarity.
                """,
            },
            "sequential_processing": {
                "type": "sequential",
                "sequence": ["summarize", "enhance"],
            },
        },
        "pipeline": {
            "markdown_task": "combined_processing",
            "chunk_task": "combined_processing",
            "batch_size": 2,
        },
        "models": {
            "default": "test_model",
            "options": [
                {
                    "name": "test_model",
                    "type": "server",
                    "endpoint": "https://tai.berkeley.edu/api/chat",
                    "api_key": "Fill out your API key here",
                }
            ],
        },
    }

    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
        yaml.safe_dump(config, tmp)
        return tmp.name


import pytest
import tempfile
import yaml
import logging

from file_conversion_router.classes.chunk import Chunk
from file_conversion_router.embedding_optimization.src.models.mock_model import (
    MockModel,
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@pytest.fixture
def mock_config_path():
    """Create a configuration file for testing with MockModel."""
    config = {
        "variables": {"max_length": 100, "style": "concise"},
        "tasks": {
            "summarize": {
                "type": "prompt",
                "prompt_template": """
                Please summarize the following text:

                $content

                Maximum length: $max_length words
                Style: $style
                """,
            },
            "enhance": {
                "type": "prompt",
                "prompt_template": """
                Enhance the following text:

                $content
                """,
            },
            "combined_processing": {
                "type": "composed",
                "subtasks": ["summarize", "enhance"],
                "final_prompt": """
                Combine these results:
                Summary: $result_summarize
                Enhanced: $result_enhance
                """,
            },
        },
        "pipeline": {
            "chunk_task": "combined_processing",
            "markdown_task": "combined_processing",
            "batch_size": 2,
        },
        "models": {
            "default": "mock_model",
            "options": [{"name": "mock_model", "type": "test_mock"}],
        },
    }

    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tmp:
        yaml.safe_dump(config, tmp)
        return tmp.name
