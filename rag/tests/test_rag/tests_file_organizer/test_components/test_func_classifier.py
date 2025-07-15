import os
import json
import pytest
from pathlib import Path
import tempfile

from file_organizer.src.config.config import FuncClassifierCfg
from file_organizer.src.core.func_classifier import FuncClassifier
from file_organizer.src.services.models import MockModel, OpenAIModel
from file_organizer.src.core.file_organizer import ContentCategory
from dotenv import load_dotenv

load_dotenv()


class TestFuncClassifier:
    @pytest.fixture
    def mock_model(self):
        return MockModel()

    @pytest.fixture
    def config(self):
        return FuncClassifierCfg()

    @pytest.fixture
    def temp_dir(self):
        with tempfile.TemporaryDirectory() as tmpdirname:
            yield Path(tmpdirname)

    @pytest.fixture
    def summaries_file(self, temp_dir):
        summaries = {
            str(temp_dir / "exercises/exercise1.md"): "Summary of exercise 1",
            str(temp_dir / "lectures/lecture1.md"): "Summary of lecture 1",
            str(temp_dir / "docs/guide1.md"): "Summary of guide 1",
            str(temp_dir / "unknown/file.md"): "Summary of an unknown file",
        }
        summaries_path = temp_dir / "summaries.json"
        with open(summaries_path, "w") as f:
            json.dump(summaries, f)
        return summaries_path

    def test_func_classifier_initialization(self, config, mock_model):
        classifier = FuncClassifier(config, mock_model)
        assert classifier.config == config
        assert classifier.model == mock_model

    @pytest.mark.asyncio
    async def test_batch_classify_with_mock_model(self, config, mock_model):
        classifier = FuncClassifier(config, mock_model)
        summaries = {"file1.md": "summary 1", "file2.md": "summary 2"}
        result = await classifier.batch_classify(summaries)
        assert isinstance(result, dict)
        assert len(result) > 0
        print("Batch classification result:", result)

    @pytest.mark.asyncio
    async def test_process_summaries_with_predefined_classification(
        self, config, mock_model, summaries_file, temp_dir
    ):
        classifier = FuncClassifier(config, mock_model)
        predefined_classification = {
            "practice": ["exercises"],
            "study": ["lectures"],
            "resource": ["docs"],
        }
        output_path = temp_dir / "functions.json"

        result = await classifier.process_summaries(
            str(summaries_file), str(output_path), predefined_classification
        )

        assert isinstance(result, dict)
        assert len(result) > 0
        assert os.path.exists(output_path)

        assert result[str(temp_dir / "exercises/exercise1.md")] == "practice"
        assert result[str(temp_dir / "lectures/lecture1.md")] == "study"
        assert result[str(temp_dir / "docs/guide1.md")] == "resource"

    @pytest.mark.asyncio
    async def test_process_summaries_with_mock_model(
        self, config, mock_model, summaries_file, temp_dir
    ):
        classifier = FuncClassifier(config, mock_model)
        output_path = temp_dir / "functions.json"

        result = await classifier.process_summaries(
            str(summaries_file), str(output_path)
        )

        assert isinstance(result, dict)
        assert len(result) > 0
        assert os.path.exists(output_path)

        with open(output_path, "r") as f:
            saved_result = json.load(f)
            assert saved_result == result

        valid_functions = {cat.value for cat in ContentCategory}
        for file_path, function in result.items():
            assert len(function) > 0

    @pytest.mark.skipif(
        not os.getenv("OPENAI_API_KEY"), reason="OpenAI API key not set"
    )
    @pytest.mark.asyncio
    async def test_batch_classify_with_openai_model(self, config, temp_dir):
        # This test now uses the real OpenAIModel
        model = OpenAIModel("gpt-4.1")
        classifier = FuncClassifier(config, model)

        summaries = {
            str(
                temp_dir / "physics_problem.md"
            ): "A set of physics problems regarding kinematics.",
            str(
                temp_dir / "history_notes.md"
            ): "Lecture notes on the French Revolution.",
        }

        result = await classifier.batch_classify(summaries)

        assert isinstance(result, dict)
        assert str(temp_dir / "physics_problem.md") in result
        assert str(temp_dir / "history_notes.md") in result

        valid_functions = {cat.value for cat in ContentCategory}
        assert result[str(temp_dir / "physics_problem.md")] == "practice"
        assert result[str(temp_dir / "history_notes.md")] == "study"

    @pytest.mark.skipif(
        not os.getenv("OPENAI_API_KEY"), reason="OpenAI API key not set"
    )
    @pytest.mark.asyncio
    async def test_process_summaries_with_openai_model(
        self, config, summaries_file, temp_dir
    ):
        model = OpenAIModel("gpt-4.1")
        classifier = FuncClassifier(config, model)
        output_path = temp_dir / "functions_openai.json"

        # We only want to test the LLM classification part, so we pass an empty predefined dict
        result = await classifier.process_summaries(
            str(summaries_file), str(output_path), predefined_classification={}
        )

        assert isinstance(result, dict)
        assert len(result) > 0
        assert os.path.exists(output_path)

        with open(output_path, "r") as f:
            saved_result = json.load(f)
            assert saved_result == result

        valid_functions = {cat.value for cat in ContentCategory}
        for file_path, function in result.items():
            assert function in valid_functions
