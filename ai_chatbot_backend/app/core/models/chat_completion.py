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

class VoiceMessage(BaseModel):
    # a message that contains audio data array in content
    role: str
    content: List[float]  # audio data as a list of floats
    tool_call_id: Optional[str] = None



class CompletionParams(BaseModel):
    course: str
    messages: List[Message]
    temperature: float
    stream: bool
    chat_type: str = "general"
    file_uuid: Optional[str] = None
    index: Optional[float] = None
    rag: Optional[bool] = True
    
class PracticeCompletionParams(BaseModel):
    course: str
    messages: List[Message]
    temperature: float
    stream: bool
    rag: Optional[bool] = True
    answer_content: Optional[str] = None
    problem_id: Optional[str] = None
    file_name: Optional[str] = None

class VoiceCompletionParams(BaseModel):
    course: str
    messages: List[Message]
    audio: VoiceMessage
    temperature: float
    stream: bool
    chat_type: str = "general"
    file_uuid: Optional[str] = None
    index: Optional[float] = None
    rag: Optional[bool] = True