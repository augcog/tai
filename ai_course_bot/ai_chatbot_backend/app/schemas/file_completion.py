# TODO: Refactor this file to reuse the schema from completion.py
# This file is currently a legacy placeholder
from pydantic import BaseModel


class CompletionResponse(BaseModel):
    completionId: str
    content: str
