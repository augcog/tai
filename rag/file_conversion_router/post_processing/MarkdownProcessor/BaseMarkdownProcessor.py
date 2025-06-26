# file: base_processor.py

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Union
import json

from openai import OpenAI
from openai.types.chat import ChatCompletionMessage


class MarkdownStructureBase(ABC):
    """
    Abstract base class for structuring markdown content from different sources.
    It defines a common workflow and handles shared functionalities like
    initializing the OpenAI client and saving responses.
    """

    def __init__(self, api_key: str, file_path: Union[str, Path], course_name: str):
        self.client = OpenAI(api_key=api_key)
        self.file_path = Path(file_path)
        self.course_name = course_name

        if not self.file_path.exists():
            raise FileNotFoundError(f"The file {self.file_path} does not exist.")

    def structure_file(self) -> Path:
        print(f"--- Starting processing for {self.file_path.name} ---")
        self._preprocess()
        print("Getting structured content from OpenAI...")
        response = self._get_structured_content()
        json_file_path = self.file_path.with_suffix('.json')
        self.save_chat_response_to_file(response, json_file_path)
        output_file_path = self.file_path.parent / f"{self.file_path.stem}_structured.md"
        print(f"Applying structure and writing to {output_file_path}...")
        self._apply_structure_to_markdown(json_file_path, output_file_path)
        print(f"--- Successfully finished processing for {self.file_path.name} ---")
        return output_file_path

    def _preprocess(self):
        pass

    @abstractmethod
    def _get_structured_content(self) -> ChatCompletionMessage:
        pass

    @abstractmethod
    def _apply_structure_to_markdown(self, json_path: Path, output_path: Path):
        pass

    def save_chat_response_to_file(self, response: ChatCompletionMessage, json_file_path: Path) -> dict:
        try:
            data = response.to_dict()
        except Exception:
            raise TypeError("Response object could not be converted to a dictionary.")
        with open(json_file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        print(f"✅ Wrote JSON response to {json_file_path}")
        return data