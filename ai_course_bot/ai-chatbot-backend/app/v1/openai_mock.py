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
    course = params.course
    print("course")
    print(course)
    # query model and get response
    formatter = local_formatter
    selector = local_selector
    parser = local_parser

    # TODO: implement the model selection and response parsing

    response = selector(formatter(params.messages), stream=params.stream, course=course)

    if params.stream:
        return StreamingResponse(parser(response), media_type="text/plain")
    else:
        return "This is a test response."


@router.post("/top_k_docs")
async def get_top_k_docs(message: str, k: int = 3, course: str = None):
    # get top k chunks
    result = top_k_selector(message, k=k, course=course)
    top_docs = result['top_docs']
    used_chunks = result['used_chunks']
    distances = result['distances']

    top_docs = top_docs if len(top_docs) > 0 else []
    used_chunks = used_chunks or []
    distances = distances if len(distances) > 0 else []

    if isinstance(top_docs, np.ndarray):
        top_docs = top_docs.tolist()

    if isinstance(used_chunks, np.ndarray):
        used_chunks = used_chunks.tolist()

    if isinstance(distances, np.ndarray):
        distances = distances.tolist()

    response_data = {
        "top_docs": top_docs,
        "used_chunks": used_chunks,
        "distances": distances
    }

    return JSONResponse(content=response_data)
