from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class MemorySynopsisDocument(BaseModel):
    """MongoDB document model for memory synopsis storage"""
    memory_synopsis_sid: str = Field(..., description="UUID generated internally for this memory synopsis")
    chat_history_sid: str = Field(..., description="Foreign key - SID of the chat history from frontend")
    content: str = Field(..., description="MemorySynopsis JSON string content")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last update timestamp")

    class Config:
        json_encoders = {
            datetime: lambda dt: dt.isoformat()
        }