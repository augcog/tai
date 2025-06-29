from unittest.mock import patch

import pytest

from rag.file_conversion_router.embedding_optimization.src.models.server_model_tai import (
    ServerModelTAI,
)


@pytest.fixture
def mock_server_response():
    """Mock server response for different inputs."""

    def mock_generate(prompt: str) -> str:
        if not prompt or prompt.strip() == "":
            raise ValueError("Empty prompt")
        if "summarize" in prompt.lower():
            return "This is a mocked summary of the content."
        if "enhance" in prompt.lower():
            return "This is enhanced version of the content."
        return "Default mocked response"

    return mock_generate


@pytest.fixture
def mock_server_model(mock_server_response):
    """Create a mocked server model."""
    with patch.object(ServerModelTAI, "generate", side_effect=mock_server_response):
        yield ServerModelTAI(endpoint="mock_endpoint", api_key="mock_key")
