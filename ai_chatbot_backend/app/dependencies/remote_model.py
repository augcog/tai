import requests


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
