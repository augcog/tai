import time

import pytest

from rag.file_conversion_router.embedding_optimization.src.models.mock_model import (
    MockModel,
)


class TestMockModel:
    """Tests for MockModel functionality."""

    def test_basic_generation(self):
        """Test basic response generation."""
        model = MockModel()
        prompt = "This is a test prompt"
        response = model.generate(prompt)

        assert "Mock response for: " in response
        assert prompt[:50] in response

    def test_empty_prompt(self):
        """Test handling of empty prompts."""
        model = MockModel()

        with pytest.raises(ValueError, match="Empty prompt provided"):
            model.generate("")

        with pytest.raises(ValueError, match="Empty prompt provided"):
            model.generate("   ")

    def test_fixed_response(self):
        """Test fixed response functionality."""
        fixed_response = "This is a fixed response"
        model = MockModel(fixed_response=fixed_response)

        assert model.generate("Any prompt") == fixed_response
        assert model.generate("Different prompt") == fixed_response

        # Test changing fixed response
        new_response = "New fixed response"
        model.set_response(new_response)
        assert model.generate("Another prompt") == new_response

    def test_failure_simulation(self):
        """Test failure simulation."""
        model = MockModel(should_fail=True)

        with pytest.raises(ValueError, match="Mock model failure"):
            model.generate("Test prompt")

        # Test toggling failure mode
        model.set_should_fail(False)
        assert model.generate("Test prompt") is not None

    def test_delay_simulation(self):
        """Test processing delay simulation."""
        delay = 0.1
        model = MockModel(delay_seconds=delay)

        start_time = time.time()
        model.generate("Test prompt")
        elapsed_time = time.time() - start_time

        assert elapsed_time >= delay

        # Test changing delay
        new_delay = 0.2
        model.set_delay(new_delay)

        start_time = time.time()
        model.generate("Test prompt")
        elapsed_time = time.time() - start_time

        assert elapsed_time >= new_delay

    def test_call_tracking(self):
        """Test call history tracking."""
        model = MockModel(track_calls=True)
        prompts = ["First prompt", "Second prompt", "Third prompt"]

        # Generate responses
        for prompt in prompts:
            model.generate(prompt, temperature=0.7)

        # Check call history
        history = model.get_call_history()
        assert len(history) == len(prompts)

        for i, prompt in enumerate(prompts):
            assert history[i]["prompt"] == prompt
            assert history[i]["kwargs"]["temperature"] == 0.7

        # Test history clearing
        model.clear_call_history()
        assert len(model.get_call_history()) == 0

    def test_call_tracking_disabled(self):
        """Test behavior when call tracking is disabled."""
        model = MockModel(track_calls=False)
        model.generate("Test prompt")

        with pytest.raises(ValueError, match="Call tracking is disabled"):
            model.get_call_history()

    def test_configuration_methods(self):
        """Test configuration method behavior."""
        model = MockModel()

        # Test response configuration
        model.set_response("Fixed")
        assert model.generate("Test") == "Fixed"

        # Test failure configuration
        model.set_should_fail(True)
        with pytest.raises(ValueError):
            model.generate("Test")

        # Test delay configuration
        model.set_should_fail(False)
        model.set_delay(0.1)
        start_time = time.time()
        model.generate("Test")
        assert time.time() - start_time >= 0.1
