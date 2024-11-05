from abc import ABC, abstractmethod


class BaseModel(ABC):
    @abstractmethod
    def generate(self, prompt: str, **kwargs) -> str:
        pass

    # @abstractmethod
    # def generate_batch(self, prompts: List[str], **kwargs) -> List[str]:
    #     pass
