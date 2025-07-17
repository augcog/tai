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
    )
    remote_model_url: str = Field(description="URL for remote model API")

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

    # Database configuration
    DATABASE_URL: str = Field(description="Database connection URL. Defaults to SQLite in project root.")

    # Practice Database configuration
    PRACTICE_DATABASE_URL: str = Field(description="Practice Database connection URL. Defaults to SQLite in project root.")

    # Server configuration
    HOST: str = Field(default="127.0.0.1", description="Server host address")
    PORT: int = Field(default=8000, description="Server port")
    RELOAD: bool = Field(
        default=False, description="Enable auto-reload for development"
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
