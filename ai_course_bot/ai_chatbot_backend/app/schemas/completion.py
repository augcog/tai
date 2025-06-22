import time
import uuid
from typing import List, Optional, Dict, Any, Union

from pydantic import BaseModel
from pydantic import Field
import datetime
import json


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


# OpenAI-compatible schema models for Chat Completion
class ToolCallFunction(BaseModel):
    name: str
    arguments: str


class ToolCall(BaseModel):
    id: str = Field(default_factory=lambda: f"call_{uuid.uuid4().hex[:8]}")
    type: str = "function"
    function: ToolCallFunction


class ChatCompletionDelta(BaseModel):
    role: Optional[str] = None
    content: Optional[str] = None
    tool_calls: Optional[List[ToolCall]] = None


class ChatCompletionChoice(BaseModel):
    index: int
    delta: ChatCompletionDelta
    logprobs: Optional[Any] = None
    finish_reason: Optional[str] = None


class ChatCompletionChunk(BaseModel):
    id: str = Field(default_factory=lambda: f"chatcmpl-{uuid.uuid4().hex}")
    object: str = "chat.completion.chunk"
    created: int = Field(default_factory=lambda: int(time.time()))
    model: str = "custom-model"
    system_fingerprint: str = "fp_custom"
    choices: List[ChatCompletionChoice]

    @classmethod
    def create_chunk(cls, content: Optional[str] = None, role: Optional[str] = None, 
                     finish_reason: Optional[str] = None, tool_calls: Optional[List[ToolCall]] = None):
        delta = ChatCompletionDelta(
            role=role,
            content=content,
            tool_calls=tool_calls
        )
        
        choice = ChatCompletionChoice(
            index=0,
            delta=delta,
            finish_reason=finish_reason
        )
        
        return cls(choices=[choice])
        
    @classmethod
    def create_reference_tool_call(cls, title: str, url: str, number: int = None) -> ToolCall:
        """
        Create a tool call for a reference.
        
        Args:
            title: Title of the reference
            url: URL of the reference
            number: The reference number (e.g., 1 for [1]) to explicitly link the marker to the tool call
            
        Returns:
            ToolCall: A formatted tool call for adding a reference
        """
        # Generate a unique ID for the tool call
        tool_call_id = f"call_{uuid.uuid4().hex}"
        
        # Create the arguments dictionary
        arguments_dict = {
            "title": title,
            "url": url
        }
        
        # Add the number if provided
        if number is not None:
            arguments_dict["number"] = number
        
        # Convert arguments to JSON string
        arguments_json = json.dumps(arguments_dict)
        
        return ToolCall(
            id=tool_call_id,
            type="function",
            function=ToolCallFunction(
                name="add_reference",
                arguments=arguments_json
            )
        )
