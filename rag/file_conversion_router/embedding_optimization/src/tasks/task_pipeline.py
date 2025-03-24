from typing import List

from rag.file_conversion_router.embedding_optimization.src.tasks.base_tasks import BaseTask

from rag.file_conversion_router.classes.chunk import Chunk
from rag.file_conversion_router.embedding_optimization.src.models.base_model import BaseModel
from rag.file_conversion_router.embedding_optimization.src.tasks.composed_task import ComposedTask


class TaskPipeline:
    def __init__(self, tasks: List[BaseTask], model: BaseModel):
        self.tasks = tasks
        self.model = model
        self._validate_dependencies()

    def _validate_dependencies(self):
        """Validate task dependencies and check for cycles."""
        task_names = {task.name for task in self.tasks}
        for task in self.tasks:
            missing_deps = set(task.depends_on) - task_names
            if missing_deps:
                raise ValueError(f"Task {task.name} has missing dependencies: {missing_deps}")

        # Check for cycles
        self._check_cycles()

    def _check_cycles(self):
        visited = set()
        path = set()

        def dfs(task_name: str):
            if task_name in path:
                raise ValueError(f"Circular dependency detected: {task_name}")
            if task_name in visited:
                return

            path.add(task_name)
            task = next(t for t in self.tasks if t.name == task_name)
            for dep in task.depends_on:
                dfs(dep)
            path.remove(task_name)
            visited.add(task_name)

        for task in self.tasks:
            dfs(task.name)

    def _get_execution_order(self) -> List[List[BaseTask]]:
        """Return tasks grouped by execution level (can be run in parallel)."""
        executed = set()
        result = []

        while len(executed) < len(self.tasks):
            level = []
            for task in self.tasks:
                if task.name not in executed and all(dep in executed for dep in task.depends_on):
                    level.append(task)
            if not level:
                raise ValueError("Unable to resolve task dependencies")
            result.append(level)
            executed.update(task.name for task in level)

        return result

    def process(self, chunks: List[Chunk]) -> List[Chunk]:
        execution_order = self._get_execution_order()
        context = {}

        for level_tasks in execution_order:
            # Group tasks by type (sequential vs composed)
            sequential_tasks = [t for t in level_tasks if not isinstance(t, ComposedTask)]
            composed_tasks = [t for t in level_tasks if isinstance(t, ComposedTask)]

            # Process sequential tasks
            for task in sequential_tasks:
                prompts = [task.generate_prompt(chunk, context) for chunk in chunks]
                results = self.model.generate_batch(prompts)
                chunks = [
                    task.process_result(result, chunk)
                    for result, chunk in zip(results, chunks)
                ]

            # Process composed tasks
            if composed_tasks:
                prompts = []
                for chunk in chunks:
                    for task in composed_tasks:
                        prompts.append(task.generate_prompt(chunk, context))

                results = self.model.generate_batch(prompts)

                # Process results
                result_idx = 0
                for chunk in chunks:
                    for task in composed_tasks:
                        chunk = task.process_result(results[result_idx], chunk)
                        result_idx += 1

        return chunks
