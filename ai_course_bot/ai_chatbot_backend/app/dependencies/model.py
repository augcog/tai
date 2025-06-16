import json

import torch
import transformers

from .remote_model import RemoteModelClient
from ..config import settings

# Global variable to store the loaded pipeline (singleton pattern)
_model_pipeline = None

def get_local_model_pipeline():
    """Loads the local text generation model for inference.
    """
    # model_id = "meta-llama/Meta-Llama-3-8B-Instruct"
    model_id = "THUDM/GLM-4-9B-0414"
    device = 0 if torch.cuda.is_available() else -1
    pipeline = transformers.pipeline(
        "text-generation",
        model=model_id,
        model_kwargs={"torch_dtype": 'auto'},
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


def initialize_model_pipeline():
    """Initialize the model pipeline once at startup.
    
    Returns the appropriate model pipeline based on configuration.
    This should be called once during app startup.
    """
    global _model_pipeline
    if _model_pipeline is not None:
        print("⚠️  Model pipeline already initialized, returning existing instance")
        return _model_pipeline
        
    print("🚀 Initializing model pipeline...")
    mode = settings.effective_llm_mode
    print(f"📦 Using LLM mode: {mode}")
    
    if mode == "local":
        print("🔧 Loading local model pipeline...")
        _model_pipeline = get_local_model_pipeline()
        print("✅ Local model pipeline loaded successfully!")
    elif mode == "remote":
        print("🌐 Setting up remote model pipeline...")
        _model_pipeline = get_remote_model_pipeline()
        print("✅ Remote model pipeline setup successfully!")
    elif mode == "mock":
        print("🎭 Setting up mock model pipeline...")
        _model_pipeline = get_mock_model_pipeline()
        print("✅ Mock model pipeline setup successfully!")
    else:
        raise ValueError(f"Unknown effective LLM mode: {mode}")
    
    return _model_pipeline


def get_model_pipeline():
    """Returns the pre-initialized model pipeline.
    
    This function should be used after initialize_model_pipeline() has been called.
    If the pipeline hasn't been initialized, it will initialize it on first call.
    """
    global _model_pipeline
    if _model_pipeline is None:
        print("⚠️  Model pipeline not initialized, initializing now...")
        return initialize_model_pipeline()
    return _model_pipeline
