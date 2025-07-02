from typing import List, Optional, Dict
from pydantic import BaseModel



class PracticeResponse(BaseModel):
    chatAnswer: Optional[str]
