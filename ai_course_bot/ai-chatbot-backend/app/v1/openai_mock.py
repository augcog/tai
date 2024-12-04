# Import necessary components from FastAPI
from fastapi import APIRouter
from app.core.models.chat_completion import *
from typing import Any
import asyncio 
import uuid
import numpy as np
import time
from fastapi.responses import StreamingResponse, JSONResponse
from app.core.actions.model_selector import course_selection
from app.core.actions.llama_seletor import local_selector, local_parser, local_formatter, top_k_selector


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
    course=params.course
    print("course")
    print(course)
    # query model and get response
    formatter = local_formatter
    selector = local_selector
    parser = local_parser

    # TODO: implement the model selection and response parsing

    response = selector(formatter(params.messages), stream=params.stream,course=course)

    if params.stream:
        return StreamingResponse(parser(response), media_type="text/plain")
    else:
         return "This is a test response."

@router.post("/top_k_docs")
async def get_top_k_docs(message: str, k: int = 3, course: str = None):
    # get top k chunks
    result = top_k_selector(message, k=k, course=course)
    top_docs = result['top_docs']
    found_chunks = result['found_chunks']
    used_chunks = result['used_chunks']

    top_docs = top_docs or []
    found_chunks = int(found_chunks) if isinstance(found_chunks, (float, np.integer)) else found_chunks
    used_chunks = int(used_chunks) if isinstance(used_chunks, (float, np.integer)) else used_chunks

    if isinstance(top_docs, np.ndarray):
        top_docs = top_docs.tolist()

    response_data = {
        "top_docs": top_docs,
        "found_chunks": found_chunks,
        "used_chunks": used_chunks
    }

    return JSONResponse(content=response_data)
