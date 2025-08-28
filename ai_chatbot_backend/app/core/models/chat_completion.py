from typing import List, Optional, Literal
import time
from pydantic import BaseModel, Field, ConfigDict


class BaseEvt(BaseModel):
    model_config = ConfigDict(extra="forbid")
    type: str
    ts: int = Field(default_factory=lambda: int(time.time()), ge=0)

class AudioSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")
    container: Literal["raw"] = "raw"
    encoding: Literal["pcm_s16le"] = "pcm_s16le"
    sample_rate_hz: Literal[24000] = 24000
    channels: Literal[1] = 1
    bit_depth: Literal[16] = 16

class ResponseDelta(BaseEvt):
    type: Literal["response.delta"] = "response.delta"
    seq: int = Field(ge=0)
    text: Optional[str] = Field(default=None, min_length=1)
    audio_b64: Optional[str] = Field(default=None, min_length=1)  # base64 of raw PCM chunk
    audio_spec: Optional[AudioSpec] = None

class Reference(BaseModel):
    model_config = ConfigDict(extra="forbid")
    reference_idx: int = Field(ge=0)
    info_path: Optional[str] = None
    url: Optional[str] = None
    file_path: Optional[str] = None

class ResponseReference(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["response.reference"] = "response.reference"
    references: List[Reference] = Field(default_factory=list, min_items=1)

class AudioTranscript(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["audio.transcript"] = "audio.transcript"
    text: str = Field(min_length=1)

class Done(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["done"] = "done"

def sse(payload: BaseEvt) -> str:
    return f"data: {payload.model_dump_json(exclude_unset=True)}\n\n"


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
    temperature: float  # TODO: Not in use. Remove?
    stream: bool
    chat_type: str = "general"
    file_uuid: Optional[str] = None
    index: Optional[float] = None
    rag: Optional[bool] = True
    audio_response: Optional[bool] = False

class FileChatCompletionParams(BaseModel):
    file_id: str
    course: str
    messages: List[Message]
    temperature: Optional[float] = 0.0  # TODO: Not in use. Remove?
    stream: bool
    selected_text: Optional[str] = None
    index: Optional[float] = None
    rag: Optional[bool] = True
    audio_response: Optional[bool] = False  # Not supported yet

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
    audio_response: Optional[bool] = False

class VoiceTranscriptParams(BaseModel):
    audio: VoiceMessage
    stream: bool = True
    sample_rate: int = 24000

class TextToSpeechParams(BaseModel):
    text: str
    stream: bool = True
    class_code: Optional[str] = None  # e.g., "CS101"
    file: Optional[str] = None  # e.g., "output.wav"
    file_idx: Optional[float] = None  # e.g., 0.0