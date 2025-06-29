import logging
from typing import Dict, Any, Optional, List

from .base_model import BaseModel

logger = logging.getLogger(__name__)


class MockModel(BaseModel):
    """Mock model for testing purposes that simulates a language model's behavior."""

    def __init__(
        self,
        should_fail: bool = False,
        fixed_response: Optional[str] = None,
        delay_seconds: float = 0,
        track_calls: bool = True,
    ):
        """
        Initialize mock model with configurable behavior.

        Args:
            should_fail: If True, generates errors to test error handling
            fixed_response: If provided, always returns this response
            delay_seconds: Simulates processing time
            track_calls: If True, tracks call history
        """
        self.should_fail = should_fail
        self.fixed_response = fixed_response
        self.delay_seconds = delay_seconds
        self.track_calls = track_calls
        self.call_history: List[Dict[str, Any]] = []

    def generate(self, prompt: str, **kwargs) -> str:
        """
        Generate a mock response for the given prompt.

        Args:
            prompt: Input prompt
            **kwargs: Additional arguments (tracked if track_calls is True)

        Returns:
            Mock generated text

        Raises:
            ValueError: If should_fail is True or if prompt is empty
        """
        if self.track_calls:
            self.call_history.append({"prompt": prompt, "kwargs": kwargs})

        # Log the call
        logger.debug(f"MockModel.generate called with prompt: {prompt[:50]}...")

        # Handle empty input
        if not prompt or prompt.strip() == "":
            raise ValueError("Empty prompt provided")

        # Simulate failure if configured
        if self.should_fail:
            raise ValueError("Mock model failure")

        # Simulate processing delay
        if self.delay_seconds > 0:
            import time

            time.sleep(self.delay_seconds)

        # Return fixed response if provided
        if self.fixed_response is not None:
            return self.fixed_response

        # Generate mock response
        return f"Using test mock model for embedding optimization. Mock response for: {prompt[:50]}..."

    def get_call_history(self) -> List[Dict[str, Any]]:
        """
        Get the history of calls made to the model.

        Returns:
            List of dictionaries containing prompt and kwargs for each call
        """
        if not self.track_calls:
            raise ValueError("Call tracking is disabled")
        return self.call_history

    def clear_call_history(self) -> None:
        """Clear the call history."""
        self.call_history = []

    def set_response(self, response: str) -> None:
        """
        Set a fixed response for the model.

        Args:
            response: Response to return for all generate calls
        """
        self.fixed_response = response

    def set_should_fail(self, should_fail: bool) -> None:
        """
        Configure whether the model should fail.

        Args:
            should_fail: If True, generate will raise ValueError
        """
        self.should_fail = should_fail

    def set_delay(self, seconds: float) -> None:
        """
        Set the processing delay.

        Args:
            seconds: Delay in seconds
        """
        self.delay_seconds = seconds
