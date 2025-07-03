# Consolidated completions router
import time

from app.api.deps import verify_api_token
from app.core.models.chat_completion import *
from app.dependencies.model import get_model_engine
from app.services.rag_retriever import top_k_selector
from app.services.rag_selector import (
    format_chat_msg,
    generate_chat_response,
    local_parser,
)
from fastapi import APIRouter, Depends
from fastapi.responses import JSONResponse, PlainTextResponse, StreamingResponse

router = APIRouter()


def generate_data():
    for number in range(1, 51):  # Generating numbers from 1 to 100
        yield f"Number: {number}\n".encode("utf-8")  # Yields data as bytes
        # Simulate a delay, can be removed or replaced with real data fetching
        time.sleep(0.1)


@router.post("/completions")
async def create_completion(
    params: CompletionCreateParams, _: bool = Depends(verify_api_token)
):
    # Get the pre-initialized pipeline
    engine = get_model_engine()

    # select model based on params.model
    course = params.course
    formatter = format_chat_msg
    selector = generate_chat_response
    parser = local_parser

    response, reference_string = selector(
        formatter(params.messages), stream=params.stream, course=course, engine=engine
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_string), media_type="text/plain"
        )
    else:
        return PlainTextResponse(response)


@router.post("/top_k_docs")
async def get_top_k_docs(
    message: str, k: int = 3, course: str = None, _: bool = Depends(verify_api_token)
):
    # get top k chunks
    result = top_k_selector(message, k=k, course=course)
    top_docs = result["top_docs"]
    chunks_used = result["used_chunks"]

    response_data = {
        "top_docs": top_docs or [],
        "used_chunks": chunks_used if top_docs else 0,
    }

    return JSONResponse(content=response_data)
