from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

import yaml


class ConfigValidationError(Exception):
    """Raised when configuration validation fails."""

    pass


class TaskType(Enum):
    """Supported task types in the pipeline."""

    PROMPT = "prompt"
    COMPOSED = "composed"
    SEQUENTIAL = "sequential"


class ModelType(Enum):
    """Supported model types."""

    LOCAL = "local"
    SERVER = "server"
    TEST_MOCK = "test_mock"


@dataclass
class ModelConfig:
    """Model configuration settings."""

    name: str
    type: ModelType
    path: Optional[str] = None
    endpoint: Optional[str] = None
    api_key: Optional[str] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ModelConfig":
        return cls(
            name=data["name"],
            type=ModelType(data["type"]),
            path=data.get("path"),
            endpoint=data.get("endpoint"),
            api_key=data.get("api_key"),
        )

    def validate(self) -> None:
        """Validate model configuration."""
        if self.type == ModelType.LOCAL and not self.path:
            raise ConfigValidationError(f"Local model '{self.name}' requires a path")
        if self.type == ModelType.SERVER and not self.endpoint:
            raise ConfigValidationError(
                f"Server model '{self.name}' requires an endpoint"
            )


@dataclass
class TaskConfig:
    """Task configuration settings."""

    type: TaskType
    prompt_template: Optional[str] = None
    subtasks: Optional[List[str]] = None
    sequence: Optional[List[str]] = None
    final_prompt: Optional[str] = None
    depends_on: Optional[List[str]] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TaskConfig":
        return cls(
            type=TaskType(data["type"]),
            prompt_template=data.get("prompt_template"),
            subtasks=data.get("subtasks"),
            sequence=data.get("sequence"),
            final_prompt=data.get("final_prompt"),
            depends_on=data.get("depends_on", []),
        )

    def validate(self, available_tasks: List[str]) -> None:
        """Validate task configuration."""
        if self.type == TaskType.PROMPT and not self.prompt_template:
            raise ConfigValidationError("Prompt tasks require a prompt_template")

        if self.type == TaskType.COMPOSED:
            if not self.subtasks:
                raise ConfigValidationError("Composed tasks require subtasks")
            for task in self.subtasks:
                if task not in available_tasks:
                    raise ConfigValidationError(
                        f"Subtask '{task}' not found in task registry"
                    )

        if self.type == TaskType.SEQUENTIAL:
            if not self.sequence:
                raise ConfigValidationError("Sequential tasks require a sequence")
            for task in self.sequence:
                if task not in available_tasks:
                    raise ConfigValidationError(
                        f"Sequence task '{task}' not found in task registry"
                    )

        if self.depends_on:
            for task in self.depends_on:
                if task not in available_tasks:
                    raise ConfigValidationError(
                        f"Dependency '{task}' not found in task registry"
                    )


@dataclass
class PipelineSettings:
    """Pipeline settings with task assignments."""

    markdown_task: str  # Required - task for markdown processing
    chunk_task: str  # Required - task for chunk processing
    batch_size: int = 1

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PipelineSettings":
        if "markdown_task" not in data:
            raise ConfigValidationError(
                "'markdown_task' must be specified in pipeline settings"
            )
        if "chunk_task" not in data:
            raise ConfigValidationError(
                "'chunk_task' must be specified in pipeline settings"
            )

        return cls(
            markdown_task=data["markdown_task"],
            chunk_task=data["chunk_task"],
            batch_size=data.get("batch_size", 1),
        )


@dataclass
class PipelineConfig:
    """Complete pipeline configuration."""

    tasks: Dict[str, TaskConfig]
    variables: Dict[str, Any]
    pipeline_settings: PipelineSettings
    models: Dict[str, ModelConfig]
    default_model: str

    @classmethod
    def from_yaml(cls, path: str) -> "PipelineConfig":
        """Load and validate configuration from a YAML file."""
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PipelineConfig":
        """Create configuration from a dictionary."""
        # Convert task configurations
        tasks = {
            name: TaskConfig.from_dict(task_data)
            for name, task_data in data.get("tasks", {}).items()
        }

        # Convert model configurations
        models = {
            model_data["name"]: ModelConfig.from_dict(model_data)
            for model_data in data.get("models", {}).get("options", [])
        }

        # Convert pipeline settings
        pipeline_settings = PipelineSettings.from_dict(data.get("pipeline", {}))

        config = cls(
            tasks=tasks,
            variables=data.get("variables", {}),
            pipeline_settings=pipeline_settings,
            models=models,
            default_model=data.get("models", {}).get("default", ""),
        )

        config.validate()
        return config

    def validate(self) -> None:
        """Validate the complete configuration."""
        # Validate tasks
        available_tasks = list(self.tasks.keys())
        for task_name, task in self.tasks.items():
            try:
                task.validate(available_tasks)
            except ConfigValidationError as e:
                raise ConfigValidationError(
                    f"Task '{task_name}' validation failed: {str(e)}"
                )

        # Validate models
        if not self.default_model:
            raise ConfigValidationError("Default model must be specified")
        if self.default_model not in self.models:
            raise ConfigValidationError(
                f"Default model '{self.default_model}' not found in model options"
            )
        for model in self.models.values():
            model.validate()

        # Validate pipeline settings
        self._validate_pipeline_settings(available_tasks)

        # Check for cycles in task dependencies
        self._check_cycles()

    def _validate_pipeline_settings(self, available_tasks: List[str]) -> None:
        """Validate pipeline settings and task assignments."""
        # Validate both required tasks exist
        for task_type in ["markdown_task", "chunk_task"]:
            task_name = getattr(self.pipeline_settings, task_type)
            if task_name not in available_tasks:
                raise ConfigValidationError(
                    f"{task_type.replace('_', ' ').title()} '{task_name}' not found in task registry"
                )

        # Validate batch_size
        if self.pipeline_settings.batch_size < 1:
            raise ConfigValidationError("Batch size must be greater than 0")

    def _check_cycles(self) -> None:
        """Check for cycles in task dependencies."""
        visited = set()
        path = set()

        def dfs(task_name: str) -> None:
            if task_name in path:
                raise ConfigValidationError(
                    f"Circular dependency detected: {task_name}"
                )
            if task_name in visited:
                return

            path.add(task_name)
            task = self.tasks[task_name]
            for dep in task.depends_on or []:
                dfs(dep)
            path.remove(task_name)
            visited.add(task_name)

        for task_name in self.tasks:
            if task_name not in visited:
                dfs(task_name)

    def get_model_config(self, model_name: Optional[str] = None) -> ModelConfig:
        """Get configuration for a specific model or the default model."""
        name = model_name or self.default_model
        if name not in self.models:
            raise ConfigValidationError(f"Model '{name}' not found in configuration")
        return self.models[name]

    def get_task(self, task_name: str) -> TaskConfig:
        """Get task configuration by name."""
        if task_name not in self.tasks:
            raise ConfigValidationError(
                f"Task '{task_name}' not found in configuration"
            )
        return self.tasks[task_name]
