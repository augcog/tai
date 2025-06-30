from functools import reduce
from typing import Any, Dict, List, Optional

from file_conversion_router.classes.chunk import Chunk
from file_conversion_router.embedding_optimization.src.tasks.base_tasks import BaseTask


class ComposedTask(BaseTask):
    def __init__(
        self,
        name: str,
        subtasks: List[BaseTask],
        prompt_template: str,
        depends_on: Optional[List[str]] = None,
    ):
        super().__init__(name, depends_on)
        self.subtasks = subtasks
        self.prompt_template = prompt_template

    def generate_prompt(self, chunk: Chunk, context: Dict[str, Any] = None) -> str:
        subtask_prompts = [
            task.generate_prompt(chunk, context) for task in self.subtasks
        ]
        return self.prompt_template.format(
            content=chunk.content, subtask_prompts="\n".join(subtask_prompts)
        )

    def process_result(self, result: str, chunk: Chunk) -> Chunk:
        # Process the composed result through each subtask's processor
        return reduce(
            lambda c, task: task.process_result(result, c), self.subtasks, chunk
        )
