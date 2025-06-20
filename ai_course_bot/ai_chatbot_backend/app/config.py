from enum import Enum
from typing import Optional, List, Union

from pydantic import Field, validator
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
        description="LLM mode: local, remote, or mock. Defaults based on environment if not set."
    )
    remote_model_url: str = Field(
        description="URL for remote model API"
    )

    # Authentication settings
    auth_required: bool = Field(
        description="Whether authentication is required"
    )
    nextauth_secret: str = Field(
        description="NextAuth secret for JWT token verification"
    )
    nextauth_url: str = Field(
        description="NextAuth URL for authentication"
    )
    allowed_domains_str: str = Field(
        default="",
        description="Comma-separated list of allowed email domains for authentication",
        alias="allowed_domains"
    )
    dev_mode: bool = Field(
        default=False,
        description="Development mode flag"
    )

    # Data directory settings
    DATA_DIR: str = Field(
        description="Directory path for file storage"
    )

    # Server configuration
    HOST: str = Field(
        default="127.0.0.1",
        description="Server host address"
    )
    PORT: int = Field(
        default=8000,
        description="Server port"
    )
    RELOAD: bool = Field(
        default=False,
        description="Enable auto-reload for development"
    )

    @validator('auth_required', pre=True)
    def parse_auth_required(cls, value):
        if isinstance(value, str):
            # Strip comments (anything after #) and whitespace
            clean_value = value.split('#')[0].strip().lower()
            if clean_value == 'true':
                return True
            elif clean_value == 'false':
                return False
        return value

    @validator('allowed_domains_str', pre=True, always=True)
    def parse_allowed_domains(cls, value):
        if value is None:
            return ""
        if isinstance(value, str):
            # Handle string representation from .env file
            # Remove brackets and quotes if present
            clean_value = value.strip().strip('[]').replace('"', '').replace("'", '')
            return clean_value
        else:
            return str(value) if value else ""

    @property
    def allowed_domains(self) -> List[str]:
        """Convert allowed_domains_str to list."""
        if not self.allowed_domains_str:
            return []
        return [domain.strip() for domain in self.allowed_domains_str.split(',') if domain.strip()]

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

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        extra = "allow"  # Allow extra fields from .env file for Pydantic v2 compatibility
        # Prevent automatic JSON parsing of list fields
        env_nested_delimiter = None
        # Custom parsing for complex types
        arbitrary_types_allowed = True


settings = Settings()
