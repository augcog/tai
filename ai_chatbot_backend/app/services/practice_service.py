"""
Service file for practice apis
All logics related to practice.py api should be written here
"""

from typing import List, Optional, Dict

from ai_chatbot_backend.app.core.models.chat_completion import Message
from ai_chatbot_backend.app.schemas.practice import PracticeResponse
from fastapi.responses import JSONResponse, StreamingResponse


def generate_practice(fileName: str, blockInfo: str, blockContent: str, message: List[Message],
                    questionNumber: int, questionState: str) -> StreamingResponse:
    """
    Generate a streaming response for chat box based on given parameters
    """
    #todo: see api/routes/completions.py create_completion() as example to generate a streaming response
    return None

def generate_practice_template(fileName: str, blockInfo: str) -> JSONResponse:
    """
    Generate a JSON response based on file name and block info
    """
    # todo: see api/routes/completions.py get_top_k_docs() as example to generate a JSON response
    questions = None
    answers = None
    response_data = {
        "questions": questions,
        "answers": answers
    }
    return JSONResponse(content=response_data)

