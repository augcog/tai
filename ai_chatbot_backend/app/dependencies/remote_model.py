import requests
from typing import Optional, Dict, Any


class RemoteModelClient:
    """
    A simplified remote LLM client that sends only the necessary parameters
    to the remote endpoint. The remote service is expected to handle all
    processing (tokenization, prompt formatting, and generation) internally.
    """

    def __init__(self, url: str):
        self.url = url

    def __call__(self, prompt: str, **kwargs):
        payload = {
            "messages": [{"role": "user", "content": prompt}],
            "course": kwargs.get("course", "default"),
            "temperature": kwargs.get("temperature", 0.7),
            "stream": kwargs.get("stream", False),
        }

        # Add response_format if provided (for structured JSON output)
        # This enables OpenAI-style response_format parameter for guaranteed valid JSON
        response_format: Optional[Dict[str, Any]] = kwargs.get("response_format")
        if response_format is not None:
            payload["response_format"] = response_format

        stream = payload.get("stream", False)
        try:
            response = requests.post(self.url, json=payload, stream=stream, timeout=30)
            response.raise_for_status()
            if stream:
                # Return an iterator over decoded lines for streaming responses.
                return response.iter_lines(decode_unicode=True)
            else:
                # Return the full JSON response for non-streaming.
                return response.json()
        except requests.RequestException as e:
            raise Exception(f"Remote LLM service request failed: {str(e)}")
