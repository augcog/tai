import os
import pytest
from rag.file_organizer.src.services.models import get_llm, MockModel, OpenAIModel, LocalHFModel, NvidiaModel
from dotenv import load_dotenv
load_dotenv()


class TestGetModel:
    def test_load_mock_model(self):
        """Test loading MockModel from profile."""
        mock_model = get_llm("mock")
        assert isinstance(mock_model, MockModel)

    @pytest.mark.skipif(not os.getenv("OPENAI_API_KEY"), reason="OpenAI API key not set")
    def test_load_openai_model(self):
        """Test loading OpenAIModel from profile."""
        openai_model = get_llm("accurate")
        assert isinstance(openai_model, OpenAIModel)
        assert openai_model.model == "gpt-4.1"
        assert openai_model.api_key is not None

    # def test_load_local_hf_model(self):
    #     """Test loading LocalHFModel from profile."""
    #     local_model = get_llm("local")
    #     assert isinstance(local_model, LocalHFModel)
    #     assert local_model.model_id == "meta-llama/Meta-Llama-3-8B-Instruct"
    #     assert local_model.quantization == "4bit"
    #     assert local_model.max_new_tokens == 1024
    #     assert local_model.temperature == 0.7

    @pytest.mark.skipif(not os.getenv("NVIDIA_API_KEY"), reason="NVIDIA API key not set")
    def test_load_nvidia_model(self):
        """Test loading NvidiaModel from profile."""
        nvidia_model = get_llm("nvidia_llama")
        assert isinstance(nvidia_model, NvidiaModel)
        assert nvidia_model.model_name == "meta/llama3-8b-instruct"
        assert nvidia_model.temperature == 0.5
        assert nvidia_model.max_new_tokens == 1024
        assert nvidia_model.client is not None

    def test_invalid_profile(self):
        """Test that an invalid profile raises an error."""
        with pytest.raises(KeyError):
            get_llm("invalid_profile")