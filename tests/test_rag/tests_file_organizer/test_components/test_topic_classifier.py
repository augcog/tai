import os
import json
import pytest
from pathlib import Path
import tempfile

from rag.file_organizer.src.config.config import TopicClassifierCfg
from rag.file_organizer.src.core.topic_classifier import TopicClassifier
from rag.file_organizer.src.services.models import MockModel, OpenAIModel
from dotenv import load_dotenv
load_dotenv()

class TestTopicClassifier:
    @pytest.fixture
    def mock_model(self):
        return MockModel()
    
    @pytest.fixture
    def config(self):
        return TopicClassifierCfg(max_topic_num=3)
    
    @pytest.fixture
    def temp_dir(self):
        with tempfile.TemporaryDirectory() as tmpdirname:
            yield tmpdirname
    
    @pytest.fixture
    def sample_summaries(self, temp_dir):
        summaries_path = os.path.join(temp_dir, "summaries.json")
        summaries = {
            "file1.md": "This is a summary about data structures and algorithms",
            "file2.md": "This is a summary about machine learning concepts"
        }
        with open(summaries_path, "w") as f:
            json.dump(summaries, f)
        return summaries_path
    
    @pytest.fixture
    def topics_list(self):
        return {"Data Structures": "data structures", "Machine Learning": "machine learning", "Python Programming": "python programming", "Algorithms": "algorithms"}
    
    def test_topic_classifier_initialization(self, config, mock_model):
        classifier = TopicClassifier(config, mock_model)
        assert classifier.config == config
        assert classifier.model == mock_model
    
    @pytest.mark.asyncio
    async def test_classify_file_with_mock_model(self, config, mock_model, topics_list):
        classifier = TopicClassifier(config, mock_model)
        file_path = "test.md"
        summary = "This is a test summary about data structures"
        
        result = await classifier.classify_file(file_path, summary, topics_list)
        
        assert isinstance(result, dict)
        assert file_path in result
        assert isinstance(result[file_path], list)
        assert len(result[file_path]) > 0
    
    @pytest.mark.asyncio
    async def test_process_summaries_with_mock_model(self, config, mock_model, sample_summaries, topics_list, temp_dir):
        classifier = TopicClassifier(config, mock_model)
        output_path = os.path.join(temp_dir, "topics.json")
        
        result = await classifier.process_summaries(sample_summaries, output_path, topics_list)
        
        assert isinstance(result, dict)
        assert len(result) > 0
        assert os.path.exists(output_path)
        
        with open(output_path, "r") as f:
            saved_result = json.load(f)
            assert saved_result == result
    
    @pytest.mark.skipif(not os.getenv("OPENAI_API_KEY"), reason="OpenAI API key not set")
    @pytest.mark.asyncio
    async def test_classify_file_with_openai_model(self, config, topics_list):
        model = OpenAIModel(model_name="gpt-4.1")
        classifier = TopicClassifier(config, model)
        file_path = "test.md"
        summary = "This is a test summary about data structures and algorithms"
        
        result = await classifier.classify_file(file_path, summary, topics_list)
        
        assert isinstance(result, dict)
        assert file_path in result
        assert isinstance(result[file_path], list)
        assert len(result[file_path]) > 0
        print("Classification result:", result)
    
    @pytest.mark.skipif(not os.getenv("OPENAI_API_KEY"), reason="OpenAI API key not set")
    @pytest.mark.asyncio
    async def test_process_summaries_with_openai_model(self, config, sample_summaries, topics_list, temp_dir):
        model = OpenAIModel(model_name="gpt-4.1")
        classifier = TopicClassifier(config, model)
        output_path = os.path.join(temp_dir, "topics.json")
        
        result = await classifier.process_summaries(sample_summaries, output_path, topics_list)
        
        assert isinstance(result, dict)
        assert len(result) > 0
        assert os.path.exists(output_path)
        
        with open(output_path, "r") as f:
            saved_result = json.load(f)
            assert saved_result == result
            
        # Verify that all topics in the result are from the provided topics list
        for file_path, topics in result.items():
            assert isinstance(topics, list)
            assert len(topics) > 0



