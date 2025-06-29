import logging
from typing import Optional

import requests

from .base_model import BaseModel

logger = logging.getLogger(__name__)


class ServerModelTAI(BaseModel):
    def __init__(
        self,
        endpoint: str,
        user_id: str = "default",
        course_id: str = "default",
        api_key: Optional[str] = None,
        timeout: int = 30,
    ):
        """
        Initialize the ServerModel with the given endpoint and API key.

        Args:
            endpoint (str): The URL of the remote LLM server API.
            user_id (str): The user ID for the request.
            course_id (str): The course ID, default is 'default'.
            api_key (Optional[str]): API key for authentication, if required.
            timeout (int): Timeout for API requests in seconds.
        """
        self.endpoint = endpoint.rstrip("/")  # Ensure no trailing slash
        self.api_key = api_key
        self.timeout = timeout
        self.headers = self._build_headers()
        self.user_id = user_id
        self.course_id = course_id

    def _build_headers(self) -> dict:
        """
        Build the HTTP headers for the API requests.

        Returns:
            dict: A dictionary of HTTP headers.
        """
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        return headers

    def generate(self, prompt: str, **kwargs) -> str:
        """
        Generate a summary for the given text using the remote LLM server.

        Args:
            prompt (str): The text to be summarized.

        Returns:
            str: The generated summary.

        Raises:
            ValueError: If the server response is invalid or an error occurs.
        """
        # Construct the messages as expected by the backend
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": prompt},
        ]

        # Prepare the payload according to the backend's expectations
        payload = {
            "course": self.course_id,
            "messages": messages,
            "temperature": 0.7,
            "stream": False,  # Set to False for non-streaming response
            "userId": self.user_id,
        }

        try:
            response = requests.post(
                self.endpoint, json=payload, headers=self.headers, timeout=self.timeout
            )
            response.raise_for_status()  # Raises HTTPError for bad responses
            summary = response.text
            if not summary:
                logger.error("Empty response from server.")
                raise ValueError("Invalid response from server: content missing.")

            return summary.strip()

        except requests.exceptions.RequestException as e:
            logger.error(f"Request to server failed: {e}")
            raise ValueError(f"Failed to generate summary: {e}")
        except ValueError as ve:
            logger.error(f"ValueError: {ve}")
            raise
