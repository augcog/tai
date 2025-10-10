import json
from app.dependencies.remote_model import RemoteModelClient
from app.config import settings
from vllm import AsyncLLMEngine, AsyncEngineArgs
from faster_whisper import WhisperModel
import os
from sentence_transformers import SentenceTransformer

# Global variable to store the loaded pipeline (singleton pattern)
_model_engine = None
_whisper_engine = None
_embedding_engine = None


def get_local_model_engine():
    os.environ["CUDA_VISIBLE_DEVICES"] = "0,1"
    # MODEL_ID = "THUDM/GLM-4-9B-0414"
    MODEL_ID = "kaitchup/GLM-Z1-32B-0414-autoround-gptq-4bit"
    TP_SIZE = 2  # tensor_parallel_size
    GPU_UTIL = 0.65
    engine_args = AsyncEngineArgs(
        model=MODEL_ID,
        tensor_parallel_size=TP_SIZE,
        gpu_memory_utilization=GPU_UTIL
    )
    engine = AsyncLLMEngine.from_engine_args(engine_args)
    return engine

def get_whisper_model_engine():
    """Returns a Whisper model engine for audio transcription."""
    model_size = "large-v3"
    return WhisperModel(model_size, device="cuda",device_index=0, compute_type="float32")

def get_local_embedding_engine():
    return SentenceTransformer("Qwen/Qwen3-Embedding-4B", device="cuda:1",
                                      model_kwargs={"torch_dtype": "auto"},
                                      tokenizer_kwargs={"padding_side": "left"})

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


def initialize_model_engine():
    """Initialize the model pipeline once at startup.

    Returns the appropriate model pipeline based on configuration.
    This should be called once during app startup.
    """
    global _model_engine
    global _whisper_engine
    global _embedding_engine
    if _model_engine is not None:
        print("⚠️  Model pipeline already initialized, returning existing instance")
        # return _model_engine, _whisper_engine
        return _model_engine
    print("🚀 Initializing model pipeline...")
    mode = settings.effective_llm_mode
    print(f"📦 Using LLM mode: {mode}")

    if mode == "local":
        print("🔧 Loading local model pipeline...")
        _model_engine = get_local_model_engine()
        print("✅ Local model pipeline loaded successfully!")
        print("🔊 Loading Whisper model engine...")
        _whisper_engine = get_whisper_model_engine()
        print("✅ Whisper model engine loaded successfully!")
        print("📚 Loading local embedding engine...")
        _embedding_engine = get_local_embedding_engine()
        print("✅ Local embedding engine loaded successfully!")
    elif mode == "remote":
        print("🌐 Setting up remote model pipeline...")
        _model_engine = get_remote_model_pipeline()
        print("✅ Remote model pipeline setup successfully!")
    elif mode == "mock":
        print("🎭 Setting up mock model pipeline...")
        _model_engine = get_mock_model_pipeline()
        print("✅ Mock model pipeline setup successfully!")
    else:
        raise ValueError(f"Unknown effective LLM mode: {mode}")

    # return _model_engine, _whisper_engine
    return _model_engine

def get_model_engine():
    """Returns the pre-initialized model pipeline.

    This function should be used after initialize_model_pipeline() has been called.
    If the pipeline hasn't been initialized, it will initialize it on first call.
    """
    global _model_engine
    if _model_engine is None:
        print("⚠️  Model pipeline not initialized, initializing now...")
        _model_engine, _ = initialize_model_engine()
    return _model_engine

def get_whisper_engine():
    """Returns the pre-initialized Whisper model engine."""
    global _whisper_engine
    if _whisper_engine is None:
        print("⚠️  Whisper model engine not initialized, initializing now...")
        _, _whisper_engine = initialize_model_engine()
    return _whisper_engine

def get_embedding_engine():
    """Returns the pre-initialized embedding engine."""
    global _embedding_engine
    if _embedding_engine is None:
        print("⚠️  Embedding engine not initialized, initializing now...")
        _, _, _embedding_engine = initialize_model_engine()
    return _embedding_engine