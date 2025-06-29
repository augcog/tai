import os
import pytest
from rag.file_organizer.src.services.models import NvidiaModel
from dotenv import load_dotenv

from rag.file_organizer.src.config.config import load_profiles
from rag.file_organizer.src.services.models import get_llm

load_dotenv()


class TestNvidiaModel:
    @pytest.fixture
    def nvidia_model(self):
        """Create a NvidiaModel instance with test configuration."""
        return NvidiaModel(
            model_name="meta/llama3-8b-instruct",
            temperature=0.5,
            max_new_tokens=1024,
            api_key=os.getenv("NVIDIA_API_KEY"),
            base_url="https://integrate.api.nvidia.com/v1",
        )

    def test_nvidia_model_initialization(self, nvidia_model):
        """Test that the NvidiaModel initializes correctly."""
        assert nvidia_model.model_name == "meta/llama3-8b-instruct"
        assert nvidia_model.temperature == 0.5
        assert nvidia_model.max_new_tokens == 1024
        assert nvidia_model.client is not None

    @pytest.mark.asyncio
    @pytest.mark.skipif(
        not os.getenv("NVIDIA_API_KEY"), reason="NVIDIA API key not set"
    )
    async def test_chat_basic_functionality(self, nvidia_model):
        """Test basic chat functionality with a simple prompt."""
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Hello, how are you?"},
        ]

        response = await nvidia_model.chat(messages=messages)

        assert isinstance(response, str)
        assert len(response) > 0
        print(f"Response: {response}")  # For debugging purposes

    @pytest.mark.asyncio
    @pytest.mark.skipif(
        not os.getenv("NVIDIA_API_KEY"), reason="NVIDIA API key not set"
    )
    async def test_chat_with_custom_parameters(self, nvidia_model):
        """Test chat with custom temperature and max_tokens parameters."""
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Tell me a short story."},
        ]

        response = await nvidia_model.chat(
            messages=messages, temperature=0.8, max_new_tokens=500
        )

        assert isinstance(response, str)
        assert len(response) > 0

    @pytest.mark.asyncio
    @pytest.mark.skipif(
        not os.getenv("NVIDIA_API_KEY"), reason="NVIDIA API key not set"
    )
    async def test_chat_with_complex_prompt(self, nvidia_model):
        """Test chat with a more complex prompt structure."""
        messages = [
            {"role": "system", "content": "You are an expert in machine learning."},
            {
                "role": "user",
                "content": "What is the difference between supervised and unsupervised learning?",
            },
            {"role": "assistant", "content": "Let me explain the key differences."},
            {"role": "user", "content": "Can you provide examples for each?"},
        ]

        response = await nvidia_model.chat(messages=messages)

        assert isinstance(response, str)
        assert len(response) > 0

    @pytest.mark.asyncio
    @pytest.mark.skipif(
        not os.getenv("NVIDIA_API_KEY"), reason="NVIDIA API key not set"
    )
    async def test_chat_error_handling(self, nvidia_model):
        """Test error handling with invalid input."""
        with pytest.raises(Exception):
            await nvidia_model.chat(messages=[])  # Empty messages should raise an error

    @pytest.mark.asyncio
    @pytest.mark.skipif(
        not os.getenv("NVIDIA_API_KEY"), reason="NVIDIA API key not set"
    )
    async def test_chat_with_long_prompt(self, nvidia_model):
        """Test chat with a longer prompt to ensure it handles larger inputs."""
        long_prompt = "Explain the concept of " + "machine learning " * 50
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": long_prompt},
        ]

        response = await nvidia_model.chat(messages=messages)

        assert isinstance(response, str)
        assert len(response) > 0
