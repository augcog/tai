from typing import Any, Dict, List

from file_conversion_router.classes.chunk import Chunk
from file_conversion_router.embedding_optimization.src.configs.enums import TaskContext
from file_conversion_router.embedding_optimization.src.models.base_model import (
    BaseModel,
)
from file_conversion_router.embedding_optimization.src.tasks.task_runner import (
    TaskRunner,
)


class Pipeline:
    def __init__(self, config: Dict[str, Any], model: BaseModel):
        self.config = config
        self.executor = TaskRunner(config["tasks"], model)
        self.pipeline_config = config["pipeline"]

    def process(self, chunks: List[Chunk]) -> List[Chunk]:
        processed_chunks = []
        for chunk in chunks:
            context = TaskContext(
                chunk=chunk, results={}, variables=self.config.get("variables", {})
            )

            result = self.executor.execute_task(
                self.pipeline_config["main_task"], context
            )

            processed_chunk = Chunk(
                content=result,
                metadata={**chunk.metadata, "task_results": context.results},
            )
            processed_chunks.append(processed_chunk)

        return processed_chunks
