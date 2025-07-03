import os
import json
import pytest
from pathlib import Path
import tempfile

from file_organizer.src.config.config import SummarizerCfg
from file_organizer.src.core.summarizer import Summarizer
from file_organizer.src.services.models import MockModel, NvidiaModel
from dotenv import load_dotenv

load_dotenv()


class TestSummarizer:
    @pytest.fixture
    def mock_model(self):
        return MockModel()

    @pytest.fixture
    def config(self):
        return SummarizerCfg(min_chunk_size=50)

    @pytest.fixture
    def temp_dir(self):
        with tempfile.TemporaryDirectory() as tmpdirname:
            yield Path(tmpdirname)

    @pytest.fixture
    def sample_markdown_file(self, temp_dir):
        file_path = temp_dir / "test.md"
        content = """
        # Test Document

        This is a test document for summarization.
        It contains multiple paragraphs to test the chunking functionality.

        ## Section 1
        This is the first section with some content.

        ## Section 2
        This is the second section with different content.
        """
        with open(file_path, "w") as f:
            f.write(content)
        return str(file_path)

    def test_summarizer_initialization(self, config, mock_model):
        summarizer = Summarizer(config, mock_model)
        assert summarizer.config == config
        assert summarizer.model == mock_model

    @pytest.mark.asyncio
    async def test_generate_summary_with_mock_model(
        self, config, mock_model, sample_markdown_file
    ):
        summarizer = Summarizer(config, mock_model)
        result = await summarizer.generate_summary(sample_markdown_file)

        assert isinstance(result, dict)
        assert sample_markdown_file in result
        assert isinstance(result[sample_markdown_file], str)
        assert len(result[sample_markdown_file]) > 0

    @pytest.mark.asyncio
    async def test_process_folder(
        self, config, mock_model, temp_dir, sample_markdown_file
    ):
        summarizer = Summarizer(config, mock_model)
        output_path = temp_dir / "summaries.json"

        result = await summarizer.process_folder(str(temp_dir), str(output_path))

        assert isinstance(result, dict)
        assert sample_markdown_file in result
        assert output_path.exists()
        with open(output_path, "r") as f:
            loaded_summaries = json.load(f)
            assert loaded_summaries == result

    @pytest.mark.asyncio
    async def test_batch_generate_summaries(
        self, config, mock_model, sample_markdown_file
    ):
        summarizer = Summarizer(config, mock_model)
        file_paths = [sample_markdown_file]

        result = await summarizer.generate_all_summaries(file_paths)

        assert isinstance(result, dict)
        assert sample_markdown_file in result
        assert isinstance(result[sample_markdown_file], str)
        assert len(result[sample_markdown_file]) > 0

    @pytest.mark.skipif(
        not os.getenv("NVIDIA_API_KEY"), reason="Local HF model tests disabled"
    )
    @pytest.mark.asyncio
    async def test_with_local_hf_model(self, config, temp_dir, sample_markdown_file):
        model = NvidiaModel(model_name="meta/llama3-8b-instruct")
        summarizer = Summarizer(config, model)
        output_path = temp_dir / "summaries.json"

        result = await summarizer.process_folder(str(temp_dir), str(output_path))

        assert isinstance(result, dict)
        assert sample_markdown_file in result
        assert isinstance(result[sample_markdown_file], str)
        assert len(result[sample_markdown_file]) > 0
        assert output_path.exists()
