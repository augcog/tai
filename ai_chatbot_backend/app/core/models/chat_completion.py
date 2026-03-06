from typing import List, Optional, Literal
from uuid import UUID
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
    text_channel: str = None
    text: Optional[str] = Field(default=None, min_length=1)
    audio_b64: Optional[str] = Field(default=None, min_length=1)  # base64 of raw PCM chunk
    audio_spec: Optional[AudioSpec] = None
    speaker_name: Optional[str] = None

class Reference(BaseModel):
    model_config = ConfigDict(extra="forbid")
    reference_idx: int = Field(ge=0)
    info_path: Optional[str] = None
    url: Optional[str] = None
    file_path: Optional[str] = None
    file_uuid: Optional[str] = None
    chunk_index: Optional[float] = None

class PageReference(BaseModel):
    """A single reference for page content generation."""
    file_uuid: str
    chunk_index: float

class PageContentParams(BaseModel):
    """Parameters for the /page-content endpoint."""
    point: str                        # Student-facing page title
    purpose: str                      # Model-facing instruction for HOW to explain
    references: List[PageReference]   # file_uuid + chunk_index from ResponseReference
    course_code: str
    stream: bool = True

class ResponseReference(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["response.reference"] = "response.reference"
    references: List[Reference] = Field(default_factory=list, min_items=1)

class CitationOpen(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["response.citation.open"] = "response.citation.open"
    citation_id: int = Field(ge=0)
    quote_text: Optional[str] = None

class CitationClose(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["response.citation.close"] = "response.citation.close"
    citation_id: int = Field(ge=0)

class AudioTranscript(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["response.transcript"] = "response.transcript"
    text: str = Field(min_length=1)

class Done(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["done"] = "done"


# === Generate-pages pipeline events ===

class OutlineComplete(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["outline.complete"] = "outline.complete"
    outline: dict

class PageStart(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["page.start"] = "page.start"
    page_index: int = Field(ge=0)
    point: str
    purpose: str = ""

class PageBullets(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["page.bullets"] = "page.bullets"
    page_index: int = Field(ge=0)
    sub_bullets: list  # [{"point": "...", "reference_ids": [1, 2]}]

class PageBlockType(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["page.block_type"] = "page.block_type"
    page_index: int = Field(ge=0)
    block_type: str  # "readable" or "not_readable"

class PageDelta(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["page.delta"] = "page.delta"
    page_index: int = Field(ge=0)
    seq: int = Field(ge=0)
    text: str = Field(min_length=1)

class PageError(BaseEvt):
    model_config = ConfigDict(extra="forbid")
    type: Literal["page.error"] = "page.error"
    page_index: int
    error: str


def sse(payload: BaseEvt) -> str:
    return f"data: {payload.model_dump_json(exclude_unset=False)}\n\n"


class Message(BaseModel):
    role: str
    content: str
    tool_call_id: Optional[str] = None

class GeneratePagesParams(BaseModel):
    """Parameters for the /generate-pages combined pipeline."""
    course_code: str
    messages: List[Message]
    stream: bool = True
    sid: Optional[str] = None         # chat_history_sid for memory retrieval

class VoiceMessage(BaseModel):
    # a message that contains audio data array in content
    role: str
    content: List[float]  # audio data as a list of floats
    tool_call_id: Optional[str] = None

class UserFocus(BaseModel):
    file_uuid: UUID
    selected_text: str = None
    chunk_index: float = None

class GeneralCompletionParams(BaseModel):
    """
    Parameters for general chat completions.
    """
    course_code: str
    audio: VoiceMessage = None
    messages: List[Message]
    stream: bool
    chat_type: str = "general"  # e.g., "general", "file", "practice"
    rag: Optional[bool] = True
    audio_response: Optional[bool] = False
    sid: Optional[str] = None  # chat_history_sid from frontend
    tutor_mode: Optional[bool] = True  # Enable tutor mode (Bloom taxonomy, hints-first)
    json_output: Optional[bool] = True  # Enable JSON output format (derived from tutor_mode if not set)

class FileCompletionParams(BaseModel):
    """
    Parameters for file-based chat completions.
    """
    course_code: str
    audio: VoiceMessage = None
    messages: List[Message]
    stream: bool
    chat_type: str = "file"  # e.g., "general", "file", "practice"
    rag: Optional[bool] = True
    audio_response: Optional[bool] = False
    sid: Optional[str] = None  # chat_history_sid from frontend
    user_focus: UserFocus
    tutor_mode: Optional[bool] = False  # Enable tutor mode (Bloom taxonomy, hints-first)
    json_output: Optional[bool] = True  # Enable JSON output format (derived from tutor_mode if not set)

class PracticeCompletionParams(BaseModel):
    """
    Parameters for practice problem chat completions.
    """
    course_code: str
    audio: VoiceMessage = None
    messages: List[Message]
    stream: bool
    chat_type: str = "practice"  # e.g., "general", "file", "practice"
    rag: Optional[bool] = True
    audio_response: Optional[bool] = False
    sid: Optional[str] = None  # chat_history_sid from frontend
    answer_content: str
    problem_id: str
    file_path: str
    tutor_mode: Optional[bool] = False  # Enable tutor mode (Bloom taxonomy, hints-first)
    json_output: Optional[bool] = True  # Enable JSON output format (derived from tutor_mode if not set)

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