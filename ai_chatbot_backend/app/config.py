from enum import Enum
from typing import Optional

from pydantic import Field
from pydantic_settings import BaseSettings


class EnvironmentEnum(str, Enum):
    """Allowed application environments."""

    dev = "dev"
    production = "production"
    test = "test"


class LLMModeEnum(str, Enum):
    """Allowed LLM modes for inference."""

    local = "local"
    remote = "remote"
    mock = "mock"


class Settings(BaseSettings):
    """Centralized backend configuration - all values come from .env file."""

    # Environment configuration
    environment: EnvironmentEnum = Field(
        description="The application environment: dev, production, or test"
    )

    # LLM Configuration
    llm_mode: Optional[LLMModeEnum] = Field(
        default=None,
        description="LLM mode: local, remote, or mock. Defaults based on environment if not set.",
        alias="LLM_MODE"
    )
    remote_model_url: str = Field(description="URL for remote model API")

    # vLLM Server Configuration
    vllm_chat_url: str = Field(
        default="http://localhost:8001/v1",
        description="vLLM server URL for chat/responses API",
        alias="VLLM_CHAT_URL"
    )
    vllm_whisper_url: str = Field(
        default="http://localhost:8003/v1",
        description="vLLM server URL for Whisper transcription API",
        alias="VLLM_WHISPER_URL"
    )
    vllm_embedding_url: str = Field(
        default="http://localhost:8002/v1",
        description="vLLM server URL for embeddings API",
        alias="VLLM_EMBEDDING_URL"
    )
    vllm_api_key: str = Field(
        default="EMPTY",
        description="API key for vLLM servers (set 'EMPTY' if no auth required)",
        alias="VLLM_API_KEY"
    )
    vllm_tts_url: str = Field(
        default="http://localhost:8004/v1",
        description="vLLM server URL for TTS (audio generation) API",
        alias="VLLM_TTS_URL"
    )
    vllm_tts_model: str = Field(
        default="",
        description="Model ID for vLLM TTS server (leave empty to auto-detect)",
        alias="VLLM_TTS_MODEL"
    )
    vllm_chat_model: str = Field(
        default="cpatonn/Qwen3-30B-A3B-Thinking-2507-AWQ-4bit",
        description="Model ID for vLLM chat server",
        alias="VLLM_CHAT_MODEL"
    )
    vllm_whisper_model: str = Field(
        default="openai/whisper-large-v3",
        description="Model ID for vLLM Whisper server",
        alias="VLLM_WHISPER_MODEL"
    )
    vllm_embedding_model: str = Field(
        default="Qwen/Qwen3-Embedding-4B",
        description="Model ID for vLLM embedding server",
        alias="VLLM_EMBEDDING_MODEL"
    )

    admin_token: str = Field(
        description="Admin token required for course management endpoints. Must be set in .env file."
    )

    admin_username: str = Field(
        description="Admin username for course management endpoints. Must be set in .env file."
    )
    admin_password: str = Field(
        description="Admin password for course management endpoints. Must be set in .env file."
    )

    api_auth_token: str = Field(
        description="API authentication token for NextJS <-> Backend communication. Must be set in .env file."
    )

    dev_mode: bool = Field(default=False, description="Development mode flag")

    # Data directory settings
    DATA_DIR: str = Field(description="Directory path for file storage")

    # MongoDB configuration
    MONGODB_URI: str = Field(
        description="MongoDB connection URI"
    )
    MONGODB_ENABLED: bool = Field(
        default=True, 
        description="Whether to use MongoDB for cloud data storage"
    )
    # Server configuration
    HOST: str = Field(default="127.0.0.1", description="Server host address")
    PORT: int = Field(default=8000, description="Server port")
    RELOAD: bool = Field(
        default=False, description="Enable auto-reload for development"
    )
    SERVER_URL: str = Field(
        default="http://127.0.0.1:8000", 
        description="Default server URL for course configuration"
    )

    @property
    def effective_llm_mode(self) -> LLMModeEnum:
        """
        Determines the effective LLM mode.

        If `llm_mode` is explicitly set, that is used.
        Otherwise:
          - In test environment: defaults to 'mock'
          - In production environment: defaults to 'local'
          - Otherwise: defaults to 'mock'
        """
        if self.llm_mode is not None:
            return self.llm_mode
        if self.environment == EnvironmentEnum.test:
            return LLMModeEnum.mock
        elif self.environment == EnvironmentEnum.production:
            return LLMModeEnum.local
        else:
            return LLMModeEnum.mock

    @property
    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.environment == EnvironmentEnum.production

    @property
    def is_development(self) -> bool:
        """Check if running in development environment."""
        return self.environment == EnvironmentEnum.dev

    @property
    def admin_token(self) -> str:
        return self.admin_token

    @property
    def admin_username(self) -> str:
        return self.admin_username

    @property
    def admin_password(self) -> str:
        return self.admin_password

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        extra = (
            "allow"  # Allow extra fields from .env file for Pydantic v2 compatibility
        )
        # Prevent automatic JSON parsing of list fields
        env_nested_delimiter = None
        # Custom parsing for complex types
        arbitrary_types_allowed = True


settings = Settings()