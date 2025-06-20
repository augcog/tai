import os
import json
import pytest
from pathlib import Path
import tempfile

from rag.file_organizer.src.config.config import OrganizerCfg
from rag.file_organizer.src.organizer import FileOrganizer
from rag.file_organizer.src.services.models import MockModel, OpenAIModel, NvidiaModel, LocalHFModel


class TestFileOrganizer:

    config_path = "course_config_sample.yaml"

    @pytest.fixture
    def temp_dir(self):
        with tempfile.TemporaryDirectory() as tmpdirname:
            yield tmpdirname
    
    @pytest.fixture
    def sample_course(self, temp_dir):
        # Create sample course structure
        course_dir = os.path.join(temp_dir, "sample_course")
        os.makedirs(course_dir)
        
        # Create sample markdown files
        files = {
            "lecture1.md": "# Lecture 1\n\nThis is a lecture about Python.",
            "exercise1.md": "# Exercise 1\n\nComplete these tasks.",
            "guide1.md": "# Guide 1\n\nThis is a reference guide."
        }
        
        for filename, content in files.items():
            file_path = os.path.join(course_dir, filename)
            with open(file_path, "w") as f:
                f.write(content)
        
        return course_dir
    
    def test_file_organizer_initialization(self):
        organizer = FileOrganizer(self.config_path)
        assert hasattr(organizer, "_summariser")
        assert hasattr(organizer, "_topic_classifier")
        assert hasattr(organizer, "_func_classifier")
        
        # Verify that mock models are used
        assert isinstance(organizer._summariser.model, NvidiaModel)
        assert isinstance(organizer._topic_classifier.model, OpenAIModel)
        assert isinstance(organizer._func_classifier.model, OpenAIModel)
    
    @pytest.mark.asyncio
    async def test_organize_files(self, sample_course, temp_dir):
        organizer = FileOrganizer(self.config_path)
        output_dir = os.path.join(temp_dir, "output")
        
        await organizer.organize_course(sample_course, output_dir)
        
        # Verify that output files were created
        assert os.path.exists(os.path.join(sample_course, "summaries.json"))
        assert os.path.exists(os.path.join(sample_course, "topics.json"))
        assert os.path.exists(os.path.join(sample_course, "functions.json"))
        
        # Verify the content of the output files
        with open(os.path.join(sample_course, "summaries.json"), "r") as f:
            summaries = json.load(f)
            assert len(summaries) > 0
        
        with open(os.path.join(sample_course, "topics.json"), "r") as f:
            topics = json.load(f)
            assert len(topics) > 0
        
        with open(os.path.join(sample_course, "functions.json"), "r") as f:
            functions = json.load(f)
            assert len(functions) > 0
    
    def test_organize_course_creates_organized_structure(self, sample_course, temp_dir):
        # not useful because we are using mock model
        organizer = FileOrganizer(self.config_path)
        output_dir = os.path.join(temp_dir, "output")
        os.makedirs(output_dir)
        
        # Run organization
        organizer.organize_course(sample_course, output_dir)

        print(output_dir)

        # Check that files were organized into appropriate directories
        for root, dirs, files in os.walk(output_dir):
            if files:  # Only check directories that contain files
                # Verify that the directory structure follows the expected pattern
                # (topic/function)
                path_parts = os.path.relpath(root, output_dir).split(os.sep)
                if len(path_parts) == 2:  # topic/function structure
                    topic, function = path_parts
                    print(topic, function)