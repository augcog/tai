import logging
from unittest.mock import patch, Mock

import pytest
import requests

from rag.file_conversion_router.embedding_optimization.src.models.server_model_tai import (
    ServerModelTAI,
)

# Set up logging
logger = logging.getLogger(__name__)


class TestServerModelTAI:
    """Unit tests for ServerModelTAI implementation."""

    def test_initialization(self):
        """Test model initialization with different parameters."""
        # Test with minimal parameters
        model = ServerModelTAI(endpoint="https://test.endpoint/api")
        assert model.endpoint == "https://test.endpoint/api"
        assert model.user_id == "default"
        assert model.course_id == "default"
        assert model.timeout == 30
        assert model.api_key is None

        # Test with all parameters
        model = ServerModelTAI(
            endpoint="https://test.endpoint/api/",  # With trailing slash
            user_id="test_user",
            course_id="test_course",
            api_key="test_key",
            timeout=60,
        )
        assert model.endpoint == "https://test.endpoint/api"  # Trailing slash removed
        assert model.user_id == "test_user"
        assert model.course_id == "test_course"
        assert model.timeout == 60
        assert model.api_key == "test_key"

    def test_header_building(self):
        """Test header construction with and without API key."""
        # Without API key
        model = ServerModelTAI(endpoint="https://test.endpoint")
        headers = model._build_headers()
        assert headers == {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }

        # With API key
        model = ServerModelTAI(endpoint="https://test.endpoint", api_key="test_key")
        headers = model._build_headers()
        assert headers == {
            "Content-Type": "application/json",
            "Accept": "application/json",
            "Authorization": "Bearer test_key",
        }

    @patch("requests.post")
    def test_successful_generation(self, mock_post):
        """Test successful text generation."""
        # Setup mock response
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.text = "Generated text response"
        mock_post.return_value = mock_response

        model = ServerModelTAI(
            endpoint="https://test.endpoint",
            user_id="test_user",
            course_id="test_course",
        )

        result = model.generate("Test prompt")

        # Verify result
        assert result == "Generated text response"

        # Verify API call
        mock_post.assert_called_once()
        call_args = mock_post.call_args

        # Verify URL
        assert call_args[0][0] == "https://test.endpoint"

        # Verify payload
        payload = call_args[1]["json"]
        assert payload == {
            "course": "test_course",
            "messages": [
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": "Test prompt"},
            ],
            "temperature": 0.7,
            "stream": False,
            "userId": "test_user",
        }

        # Verify request parameters
        assert call_args[1]["timeout"] == 30

    @patch("requests.post")
    def test_error_handling(self, mock_post):
        """Test various error scenarios."""
        model = ServerModelTAI(endpoint="https://test.endpoint")

        # Test HTTP error
        mock_post.side_effect = requests.exceptions.HTTPError("500 Server Error")
        with pytest.raises(ValueError, match="Failed to generate summary: "):
            model.generate("Test prompt")

        # Test connection error
        mock_post.side_effect = requests.exceptions.ConnectionError("Connection failed")
        with pytest.raises(ValueError, match="Failed to generate summary: "):
            model.generate("Test prompt")

        # Test timeout error
        mock_post.side_effect = requests.exceptions.Timeout("Request timed out")
        with pytest.raises(ValueError, match="Failed to generate summary: "):
            model.generate("Test prompt")

        # Test empty response
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.text = ""
        mock_post.side_effect = None
        mock_post.return_value = mock_response

        with pytest.raises(
            ValueError, match="Invalid response from server: content missing."
        ):
            model.generate("Test prompt")

    @patch("requests.post")
    def test_whitespace_handling(self, mock_post):
        """Test handling of whitespace in responses."""
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.text = "  Response with whitespace  \n\n"
        mock_post.return_value = mock_response

        model = ServerModelTAI(endpoint="https://test.endpoint")
        result = model.generate("Test prompt")

        # Verify whitespace is stripped
        assert result == "Response with whitespace"

    @patch("requests.post")
    def test_logging(self, mock_post, caplog):
        """Test logging functionality."""
        model = ServerModelTAI(endpoint="https://test.endpoint")

        # Test error logging
        mock_post.side_effect = requests.exceptions.ConnectionError("Connection failed")

        with caplog.at_level(logging.ERROR):
            with pytest.raises(ValueError):
                model.generate("Test prompt")

            assert "Request to server failed" in caplog.text

        # Test empty response logging
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.text = ""
        mock_post.side_effect = None
        mock_post.return_value = mock_response

        with caplog.at_level(logging.ERROR):
            with pytest.raises(ValueError):
                model.generate("Test prompt")

            assert "Empty response from server" in caplog.text
