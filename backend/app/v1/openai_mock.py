# Import necessary components from FastAPI
from fastapi import APIRouter, HTTPException
from app.core.models.chat_completion import *
from typing import Any
import asyncio
import uuid
import time
from fastapi.responses import StreamingResponse, PlainTextResponse
from app.core.actions.model_selector import course_selection
from app.api.v1.services.rag_selector import generate_chat_response, local_parser, format_chat_msg
from app.api.v1.services.rag_retriever import top_k_selector
import httpx
import requests


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
    stream = params.stream
    print("course")
    print(course)
    formatter = format_chat_msg
    selector = generate_chat_response
    parser = local_parser

    response,reference_string = selector(formatter(params.messages), stream=stream, course=course)

    if params.stream:
        return StreamingResponse(parser(response,reference_string), media_type="text/plain")
    else:
         return PlainTextResponse(response)

@router.post("/top_k_docs")
async def get_top_k_docs(message: str, k: int = 3, course: str = None):
    # get top k chunks
    result = top_k_selector(message, k=k, course=course)
    top_docs = result['top_docs']
    chunks_used = result['used_chunks']

    response_data = {
        "top_docs": top_docs or [],
        "used_chunks": chunks_used if top_docs else 0
    }

    return JSONResponse(content=response_data)
