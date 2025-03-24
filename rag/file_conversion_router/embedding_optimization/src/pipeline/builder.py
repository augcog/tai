from pathlib import Path
from typing import Tuple

from rag.file_conversion_router.embedding_optimization.src.configs.pipeline_config import (
    PipelineConfig,
    ModelConfig,
    ModelType,
    ConfigValidationError
)
from rag.file_conversion_router.embedding_optimization.src.models.base_model import BaseModel
from rag.file_conversion_router.embedding_optimization.src.models.local_model import LocalLLama3Model
from rag.file_conversion_router.embedding_optimization.src.models.mock_model import MockModel
from rag.file_conversion_router.embedding_optimization.src.models.server_model_tai import ServerModelTAI
from rag.file_conversion_router.embedding_optimization.src.tasks.task_runner import TaskRunner


class PipelineBuilder:
    """
    Handles pipeline configuration loading and model initialization.
    Acts as a factory for creating pipeline components.
    """

    @staticmethod
    def load_config(config_path: str) -> PipelineConfig:
        """Load and validate pipeline configuration from YAML file."""
        if not Path(config_path).exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")

        return PipelineConfig.from_yaml(config_path)

    @staticmethod
    def create_model(model_config: ModelConfig) -> BaseModel:
        """
        Create and initialize model instance based on configuration.

        Args:
            model_config: ModelConfig instance

        Returns:
            Initialized model instance

        Raises:
            ConfigValidationError: If model configuration is invalid
        """
        if model_config.type == ModelType.TEST_MOCK:
            return MockModel()

        elif model_config.type == ModelType.LOCAL:
            if not model_config.path:
                raise ConfigValidationError("Local model requires 'path' in configuration")
            return LocalLLama3Model(
                model_path=model_config.path,
                model_name=model_config.name
            )

        elif model_config.type == ModelType.SERVER:
            if not model_config.endpoint:
                raise ConfigValidationError("Server model requires 'endpoint' in configuration")
            return ServerModelTAI(
                endpoint=model_config.endpoint,
                api_key=model_config.api_key
            )

        else:
            raise ConfigValidationError(f"Unknown model type: {model_config.type}")

    @classmethod
    def create_task_runner(cls, config: PipelineConfig, model: BaseModel) -> TaskRunner:
        """Create and initialize TaskRunner instance."""
        return TaskRunner(
            task_registry=config.tasks,
            model=model
        )

    @classmethod
    def build(cls, config_path: str) -> Tuple[PipelineConfig, BaseModel, TaskRunner]:
        """
        Build complete pipeline components from configuration file.

        Args:
            config_path: Path to configuration file

        Returns:
            Tuple of (PipelineConfig, BaseModel, TaskRunner)

        Raises:
            ConfigValidationError: If configuration is invalid
        """
        config = cls.load_config(config_path)

        model_config = config.get_model_config()

        model = cls.create_model(model_config)

        task_runner = cls.create_task_runner(config, model)

        return config, model, task_runner
