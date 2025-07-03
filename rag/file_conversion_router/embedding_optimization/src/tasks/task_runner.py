import logging
from dataclasses import dataclass
from string import Template
from typing import Dict, Any, Optional

from file_conversion_router.embedding_optimization.src.configs.pipeline_config import (
    TaskType,
    TaskConfig,
)
from file_conversion_router.embedding_optimization.src.models.base_model import (
    BaseModel,
)
from file_conversion_router.embedding_optimization.src.tasks.task_context import (
    TaskContext,
)

logger = logging.getLogger(__name__)


class TaskExecutionError(Exception):
    """Raised when task execution fails."""

    pass


@dataclass
class TaskResult:
    """Contains the result of a task execution."""

    content: str
    metadata: Dict[str, Any]


class TaskRunner:
    """
    Handles task execution and management for the pipeline.
    Supports prompt, composed, and sequential tasks.
    """

    def __init__(self, task_registry: Dict[str, TaskConfig], model: BaseModel):
        """
        Initialize TaskRunner with task registry and model.

        Args:
            task_registry: Dictionary mapping task names to their configurations
            model: Model instance to use for generating content
        """
        self.task_registry = task_registry
        self.model = model
        self._validate_task_registry()

    def _validate_task_registry(self) -> None:
        """Validate task registry for consistency and cycles."""
        visited = set()
        path = set()

        def check_cycles(task_name: str) -> None:
            if task_name in path:
                raise TaskExecutionError(f"Circular dependency detected: {task_name}")
            if task_name in visited:
                return

            path.add(task_name)
            task = self.task_registry[task_name]

            # Check dependencies in subtasks
            if task.type == TaskType.COMPOSED and task.subtasks:
                for subtask in task.subtasks:
                    if subtask not in self.task_registry:
                        raise TaskExecutionError(
                            f"Subtask '{subtask}' not found in registry"
                        )
                    check_cycles(subtask)

            # Check dependencies in sequence
            if task.type == TaskType.SEQUENTIAL and task.sequence:
                for seq_task in task.sequence:
                    if seq_task not in self.task_registry:
                        raise TaskExecutionError(
                            f"Sequence task '{seq_task}' not found in registry"
                        )
                    check_cycles(seq_task)

            # Check explicit dependencies
            if task.depends_on:
                for dep in task.depends_on:
                    if dep not in self.task_registry:
                        raise TaskExecutionError(
                            f"Dependency '{dep}' not found in registry"
                        )
                    check_cycles(dep)

            path.remove(task_name)
            visited.add(task_name)

        # Check each task in the registry
        for task_name in self.task_registry:
            if task_name not in visited:
                check_cycles(task_name)

    def execute_task(self, task_id: str, context: TaskContext) -> str:
        """
        Execute a task by its ID with given context.

        Args:
            task_id: Identifier of the task to execute
            context: Execution context containing chunk and variables

        Returns:
            Processed content as string

        Raises:
            TaskExecutionError: If task execution fails
        """
        if task_id not in self.task_registry:
            raise TaskExecutionError(f"Task '{task_id}' not found in registry")

        try:
            task_config = self.task_registry[task_id]

            if task_config.type == TaskType.PROMPT:
                return self._execute_prompt_task(task_config, context)
            elif task_config.type == TaskType.COMPOSED:
                return self._execute_composed_task(task_config, context)
            elif task_config.type == TaskType.SEQUENTIAL:
                return self._execute_sequential_task(task_config, context)
            else:
                raise TaskExecutionError(f"Unknown task type: {task_config.type}")

        except Exception as e:
            raise TaskExecutionError(
                f"Failed to execute task '{task_id}': {str(e)}"
            ) from e

    def _execute_prompt_task(
        self, task_config: TaskConfig, context: TaskContext
    ) -> str:
        """Execute a prompt-based task."""
        try:
            # Create template from the prompt template string
            template = Template(task_config.prompt_template)

            # Prepare variables for template substitution
            template_vars = {
                "content": context.chunk.content,
                **context.variables,
                **{f"result_{k}": v for k, v in context.results.items()},
            }

            # Generate the prompt
            prompt = template.safe_substitute(template_vars)

            # Log the generated prompt for debugging
            logger.debug(f"Generated prompt: {prompt}")

            # Generate content using the model
            return self.model.generate(prompt)

        except Exception as e:
            raise TaskExecutionError(f"Failed to execute prompt task: {str(e)}") from e

    def _execute_composed_task(
        self, task_config: TaskConfig, context: TaskContext
    ) -> str:
        """Execute a composed task that combines multiple subtasks."""
        try:
            # Execute all subtasks
            subtask_results = {}
            for subtask_id in task_config.subtasks:
                result = self.execute_task(subtask_id, context)
                subtask_results[subtask_id] = result
                context.results[subtask_id] = result

            # If there's a final prompt, use it to combine results
            if task_config.final_prompt:
                context.variables.update(subtask_results)
                final_config = TaskConfig(
                    type=TaskType.PROMPT, prompt_template=task_config.final_prompt
                )
                return self._execute_prompt_task(final_config, context)

            # Otherwise, return the last subtask's result
            return subtask_results[task_config.subtasks[-1]]

        except Exception as e:
            raise TaskExecutionError(
                f"Failed to execute composed task: {str(e)}"
            ) from e

    def _execute_sequential_task(
        self, task_config: TaskConfig, context: TaskContext
    ) -> str:
        """Execute a sequence of tasks in order."""
        try:
            result = context.chunk.content

            # Execute each task in sequence
            for task_id in task_config.sequence:
                # Update context with current content
                context.chunk.content = result

                # Execute next task
                result = self.execute_task(task_id, context)

                # Store result in context
                context.results[task_id] = result

            return result

        except Exception as e:
            raise TaskExecutionError(
                f"Failed to execute sequential task: {str(e)}"
            ) from e

    def get_task_info(self, task_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a specific task.

        Args:
            task_id: Task identifier

        Returns:
            Dictionary containing task information or None if task not found
        """
        if task_id not in self.task_registry:
            return None

        task = self.task_registry[task_id]
        return {
            "type": task.type.value,
            "has_prompt": bool(task.prompt_template),
            "subtasks": task.subtasks,
            "sequence": task.sequence,
            "has_final_prompt": bool(task.final_prompt),
            "dependencies": task.depends_on,
        }
