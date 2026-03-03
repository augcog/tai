import json
from openai import OpenAI, AsyncOpenAI
from app.dependencies.remote_model import RemoteModelClient
from app.dependencies.openai_model import OpenAIModelClient
from app.config import settings

# Global variable to store the loaded clients (singleton pattern)
_model_engine = None
_whisper_engine = None
_embedding_engine = None
# Model IDs from config (with fallback defaults)
LLM_MODEL_ID = settings.vllm_chat_model


def get_vllm_chat_client():
    """Returns an AsyncOpenAI client configured for vLLM chat/responses API."""
    return AsyncOpenAI(
        base_url=settings.vllm_chat_url,
        api_key=settings.vllm_api_key
    )


def get_vllm_whisper_client():
    """Returns an OpenAI client configured for vLLM Whisper transcription API."""
    return OpenAI(
        base_url=settings.vllm_whisper_url,
        api_key=settings.vllm_api_key
    )


def get_vllm_embedding_client():
    """Returns an OpenAI client configured for vLLM embeddings API."""
    return OpenAI(
        base_url=settings.vllm_embedding_url,
        api_key=settings.vllm_api_key
    )


def get_remote_model_pipeline():
    """Returns a pipeline that sends inference requests to a remote service.

    Tokenization is performed locally to maintain a consistent interface.
    """
    return RemoteModelClient(url=settings.remote_model_url)


def get_openai_model_pipeline():
    """Returns an OpenAI API client for inference.

    Uses OpenAI's native structured output support for guaranteed valid JSON.
    Requires OPENAI_API_KEY and optionally OPENAI_MODEL in environment.
    """
    return OpenAIModelClient(
        api_key=settings.openai_api_key,
        model=settings.openai_model
    )


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
    """Initialize the model clients once at startup.

    Returns the appropriate model client based on configuration.
    This should be called once during app startup.
    """
    global _model_engine
    global _whisper_engine
    global _embedding_engine
    if _model_engine is not None:
        print("⚠️  Model pipeline already initialized, returning existing instance")
        return _model_engine
    print("🚀 Initializing model pipeline...")
    mode = settings.effective_llm_mode
    print(f"📦 Using LLM mode: {mode}")

    if mode == "local":
        print("🔧 Connecting to vLLM chat server...")
        _model_engine = get_vllm_chat_client()
        print(f"✅ Connected to vLLM chat server at {settings.vllm_chat_url}")
        print("🔊 Connecting to vLLM Whisper server...")
        _whisper_engine = get_vllm_whisper_client()
        print(f"✅ Connected to vLLM Whisper server at {settings.vllm_whisper_url}")
        print("📚 Connecting to vLLM embedding server...")
        _embedding_engine = get_vllm_embedding_client()
        print(f"✅ Connected to vLLM embedding server at {settings.vllm_embedding_url}")
    elif mode == "remote":
        print("🌐 Setting up remote model pipeline...")
        _model_engine = get_remote_model_pipeline()
        print("✅ Remote model pipeline setup successfully!")
        print("📚 Loading vLLM embedding engine for RAG...")
        _embedding_engine = get_vllm_embedding_client()
        print(f"✅ vLLM embedding engine loaded successfully at {settings.vllm_embedding_url}")
    elif mode == "openai":
        print("🌐 Setting up OpenAI model pipeline...")
        _model_engine = get_openai_model_pipeline()
        print(f"✅ OpenAI model pipeline setup successfully! (model: {settings.openai_model})")
        print("📚 Loading vLLM embedding engine for RAG...")
        _embedding_engine = get_vllm_embedding_client()
        print(f"✅ vLLM embedding engine loaded successfully at {settings.vllm_embedding_url}")
    elif mode == "mock":
        print("🎭 Setting up mock model pipeline...")
        _model_engine = get_mock_model_pipeline()
        print("✅ Mock model pipeline setup successfully!")
    else:
        raise ValueError(f"Unknown effective LLM mode: {mode}")

    return _model_engine


def get_model_engine():
    """Returns the pre-initialized model client.

    This function should be used after initialize_model_pipeline() has been called.
    If the pipeline hasn't been initialized, it will initialize it on first call.
    """
    global _model_engine
    if _model_engine is None:
        print("⚠️  Model pipeline not initialized, initializing now...")
        initialize_model_engine()
    return _model_engine


_override_engines: dict = {}


def get_engine_for_mode(mode: str):
    """Get a cached engine for the given mode."""
    # If it matches startup mode, return the default singleton
    if mode == settings.effective_llm_mode:
        return get_model_engine()
    if mode in _override_engines:
        return _override_engines[mode]
    # Create and cache
    if mode == "openai":
        engine = get_openai_model_pipeline()
    elif mode == "local":
        engine = get_vllm_chat_client()
    else:
        raise ValueError(f"Unsupported mode: {mode}")
    _override_engines[mode] = engine
    print(f"[INFO] Created override engine for mode: {mode}")
    return engine


def get_whisper_engine():
    """Returns the pre-initialized Whisper client."""
    global _whisper_engine
    if _whisper_engine is None:
        print("⚠️  Whisper client not initialized, initializing now...")
        initialize_model_engine()
    return _whisper_engine


def get_embedding_engine():
    """Returns the pre-initialized embedding client."""
    global _embedding_engine
    if _embedding_engine is None:
        print("⚠️  Embedding client not initialized, initializing now...")
        initialize_model_engine()
    return _embedding_engine
