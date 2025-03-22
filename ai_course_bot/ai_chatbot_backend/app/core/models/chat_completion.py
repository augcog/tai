from typing import List, Optional, Dict

from pydantic import BaseModel


class CompletionChoiceLogprobs(BaseModel):
    text_offset: Optional[List[int]] = None
    token_logprobs: Optional[List[float]] = None
    tokens: Optional[List[str]] = None
    top_logprobs: Optional[List[Dict[str, float]]] = None


class CompletionChoice(BaseModel):
    finish_reason: str
    index: int
    logprobs: Optional[CompletionChoiceLogprobs] = None
    text: str


class CompletionUsage(BaseModel):
    completion_tokens: int = 100
    prompt_tokens: int = 100
    total_tokens: int = 200


class Completion(BaseModel):
    id: str
    choices: List[CompletionChoice]
    created: int
    model: str
    object: str = "text_completion"
    usage: Optional[CompletionUsage] = None


class Message(BaseModel):
    role: str
    content: str
    tool_call_id: Optional[str] = None


class CompletionCreateParams(BaseModel):
    course: str
    messages: Optional[List[Message]]
    temperature: float
    stream: bool
    rag: Optional[bool] = True
