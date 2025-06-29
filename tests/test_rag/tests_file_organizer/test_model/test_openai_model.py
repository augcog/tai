import os
import pytest
from rag.file_organizer.src.services.models import OpenAIModel
from dotenv import load_dotenv

load_dotenv()


class TestOpenAIModel:
    @pytest.fixture
    def openai_model(self):
        """Create an OpenAIModel instance with test configuration."""
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            pytest.skip("OpenAI API key not set")
        return OpenAIModel(api_key=api_key, model_name="gpt-3.5-turbo")

    @pytest.mark.asyncio
    async def test_openai_model_initialization(self, openai_model):
        """Test that the OpenAIModel initializes correctly."""
        assert openai_model.model == "gpt-3.5-turbo"
        assert openai_model.api_key is not None

    @pytest.mark.asyncio
    async def test_chat_basic_functionality(self, openai_model):
        """Test basic chat functionality with a simple prompt."""
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Say hello in one word."},
        ]

        response = await openai_model.chat(messages=messages)
        print(response)

        assert isinstance(response, str)
        assert len(response) > 0
        assert (
            len(response.split()) <= 2
        )  # Should be one word plus possible punctuation

    @pytest.mark.asyncio
    async def test_chat_with_empty_messages(self, openai_model):
        """Test chat with empty messages list."""
        with pytest.raises(Exception):
            await openai_model.chat(messages=[])

    @pytest.mark.asyncio
    async def test_chat_with_invalid_messages(self, openai_model):
        """Test chat with invalid message format."""
        with pytest.raises(Exception):
            await openai_model.chat(messages=[{"invalid": "format"}])
