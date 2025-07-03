from typing import Optional
from pydantic import BaseModel



class PracticeResponse(BaseModel):
    """
    TODO: after testing vs code extension and practice api successfully, should modify/delete this file
    response for practice services, may not use, will delete or modify later
    """
    chatAnswer: Optional[str]
