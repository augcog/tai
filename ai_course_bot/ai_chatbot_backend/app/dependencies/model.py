import json

import torch
import transformers

from .remote_model import RemoteModelClient
from ..config import settings


def get_local_model_pipeline():
    """Loads the local text generation model for inference.
    """
    model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
    device = 0 if torch.cuda.is_available() else -1
    pipeline = transformers.pipeline(
        "text-generation",
        model=model_id,
        model_kwargs={"torch_dtype": torch.bfloat16},
        device=device
    )
    return pipeline


def get_remote_model_pipeline():
    """Returns a pipeline that sends inference requests to a remote service.

    Tokenization is performed locally to maintain a consistent interface.
    """
    return RemoteModelClient(url=settings.remote_model_url)


def get_mock_model_pipeline():
    """
    Returns a mock pipeline that simulates LLM responses for testing and development.

    In this mock implementation, no tokenizer is loaded and the model does not perform
    any real inference. Instead, it returns a simulated response based on the input prompt.

    The output format is made to resemble the remote service's response:
      - If streaming is enabled, returns a generator yielding newline-delimited JSON strings.
      - Otherwise, returns a complete JSON response as a Python dict.
    """

    class MockPipeline:
        def __call__(self, prompt: str, **kwargs):
            max_length = kwargs.get("max_length", None)

            def stream_generator():
                nonlocal max_length
                # Simulate token-by-token streaming response.
                simulated_text = "Simulated complete response for: " + prompt
                tokens = simulated_text.split()
                if max_length is not None:
                    tokens = tokens[:max_length]
                # Yield each token as a NDJSON message.
                for token in tokens:
                    yield json.dumps({"type": "token", "data": token + " "}) + "\n"

            return stream_generator()

    return MockPipeline()


def get_model_pipeline():
    """Returns the appropriate model pipeline based on configuration.

    Modes:
      - local: Run inference locally.
      - remote: Call a remote LLM service.
      - mock: Return simulated responses.

    The effective mode is determined by the overall environment (env) unless llm_mode is explicitly set.
    """
    mode = settings.effective_llm_mode
    if mode == "local":
        return get_local_model_pipeline()
    elif mode == "remote":
        return get_remote_model_pipeline()
    elif mode == "mock":
        return get_mock_model_pipeline()
    else:
        raise ValueError(f"Unknown effective LLM mode: {mode}")
