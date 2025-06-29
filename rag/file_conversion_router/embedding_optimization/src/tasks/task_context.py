import copy
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Any, Optional, TypeVar, Generic

from rag.file_conversion_router.classes.chunk import Chunk

T = TypeVar("T")


class ContextVariable(Generic[T]):
    """
    Wrapper for context variables with metadata and validation.
    Allows tracking when variables are set and accessed.
    """

    def __init__(self, value: T, description: Optional[str] = None):
        self._value = value
        self._description = description
        self._last_updated = datetime.now()
        self._access_count = 0

    @property
    def value(self) -> T:
        """Get the variable value and track access."""
        self._access_count += 1
        return self._value

    @value.setter
    def value(self, new_value: T) -> None:
        """Set the variable value and update metadata."""
        self._value = new_value
        self._last_updated = datetime.now()

    @property
    def metadata(self) -> Dict[str, Any]:
        """Get variable metadata."""
        return {
            "description": self._description,
            "last_updated": self._last_updated,
            "access_count": self._access_count,
            "type": type(self._value).__name__,
        }


@dataclass
class TaskContext:
    """
    Maintains the context for task execution including the current chunk,
    intermediate results, and shared variables.

    Attributes:
        chunk: The current chunk being processed
        results: Dictionary storing intermediate results from previous tasks
        variables: Dictionary storing shared variables across tasks
        metadata: Additional metadata about the context
        _history: Internal tracking of context changes
    """

    chunk: Chunk
    results: Dict[str, Any] = field(default_factory=dict)
    variables: Dict[str, ContextVariable] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(
        default_factory=lambda: {"created_at": datetime.now(), "updates": 0}
    )
    _history: Dict[str, list] = field(
        default_factory=lambda: {"variables": [], "results": []}
    )

    def set_variable(
        self, name: str, value: Any, description: Optional[str] = None
    ) -> None:
        """
        Set a context variable with optional description.

        Args:
            name: Variable name
            value: Variable value
            description: Optional description of the variable's purpose
        """
        var = ContextVariable(value, description)
        self.variables[name] = var
        self._history["variables"].append(
            {"name": name, "value": value, "timestamp": datetime.now()}
        )
        self.metadata["updates"] += 1

    def get_variable(self, name: str, default: Optional[Any] = None) -> Any:
        """
        Get a context variable value with optional default.

        Args:
            name: Variable name
            default: Default value if variable not found

        Returns:
            Variable value or default if not found
        """
        var = self.variables.get(name)
        return var.value if var is not None else default

    def add_result(self, task_name: str, result: Any) -> None:
        """
        Add a task result to the context.

        Args:
            task_name: Name of the task
            result: Task execution result
        """
        self.results[task_name] = result
        self._history["results"].append(
            {"task": task_name, "timestamp": datetime.now()}
        )
        self.metadata["updates"] += 1

    def get_result(self, task_name: str, default: Optional[Any] = None) -> Any:
        """
        Get a task result with optional default.

        Args:
            task_name: Name of the task
            default: Default value if result not found

        Returns:
            Task result or default if not found
        """
        return self.results.get(task_name, default)

    def get_template_variables(self) -> Dict[str, Any]:
        """
        Get all variables formatted for template substitution.

        Returns:
            Dictionary of variables ready for template use
        """
        template_vars = {
            "content": self.chunk.content,
            **{name: var.value for name, var in self.variables.items()},
            **{f"result_{k}": v for k, v in self.results.items()},
        }
        return template_vars

    def create_child_context(self) -> "TaskContext":
        """
        Create a new context inheriting from this one.
        Useful for subtasks that need isolated context.

        Returns:
            New TaskContext instance with copied data
        """
        return TaskContext(
            chunk=copy.deepcopy(self.chunk),
            results=copy.deepcopy(self.results),
            variables={
                name: ContextVariable(var.value, var._description)
                for name, var in self.variables.items()
            },
            metadata={"parent_context_id": id(self), "created_at": datetime.now()},
        )

    def merge_child_context(self, child_context: "TaskContext") -> None:
        """
        Merge results and variables from a child context.

        Args:
            child_context: Child context to merge
        """
        self.results.update(child_context.results)
        for name, var in child_context.variables.items():
            if (
                name not in self.variables
                or var._last_updated > self.variables[name]._last_updated
            ):
                self.variables[name] = var

    def get_execution_summary(self) -> Dict[str, Any]:
        """
        Get a summary of context execution history.

        Returns:
            Dictionary containing execution summary
        """
        return {
            "total_updates": self.metadata["updates"],
            "variable_updates": len(self._history["variables"]),
            "result_updates": len(self._history["results"]),
            "execution_time": (
                datetime.now() - self.metadata["created_at"]
            ).total_seconds(),
            "variables": {name: var.metadata for name, var in self.variables.items()},
        }

    def clear_history(self) -> None:
        """Clear execution history to free memory."""
        self._history = {"variables": [], "results": []}
