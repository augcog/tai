# Import necessary components from FastAPI
from fastapi import APIRouter
from app.core.models.chat_completion import *
from typing import Any
import asyncio 
import uuid
import time
from fastapi.responses import StreamingResponse
from app.core.actions.model_selector import course_selection
from app.core.actions.llama_seletor import local_selector, local_parser, local_formatter

def generate_data():
        for number in range(1, 51):  # Generating numbers from 1 to 100
            yield f"Number: {number}\n".encode("utf-8")  # Yields data as bytes
            # Simulate a delay, can be removed or replaced with real data fetching
            time.sleep(0.1)
# Create an API router
router = APIRouter(prefix="/api/chat")

@router.post("/completions")
async def create_completion(params: CompletionCreateParams):
    # select model based on params.model
    course_model_address = course_selection.get(params.course, "default")
<<<<<<< HEAD
    
=======
    course=params.course
    print("course")
    print(course)
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
    # query model and get response
    formatter = local_formatter
    selector = local_selector
    parser = local_parser

    # TODO: implement the model selection and response parsing

<<<<<<< HEAD
    response = selector(formatter(params.messages), stream=params.stream)
=======
    response = selector(formatter(params.messages), stream=params.stream,course=course)
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc

    if params.stream:
        return StreamingResponse(parser(response), media_type="text/plain")
    else:
         return "This is a test response."