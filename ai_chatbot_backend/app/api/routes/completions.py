# Consolidated completions router
import time

from app.api.deps import verify_api_token
from app.core.models.chat_completion import *
from app.dependencies.model import get_model_engine, get_whisper_engine
from app.services.rag_retriever import top_k_selector
from app.services.rag_selector import (
    format_chat_msg,
    generate_chat_response,
    generate_practice_response,
    local_parser,
)
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse, PlainTextResponse, StreamingResponse
from sqlalchemy.orm import Session
from app.core.dbs.metadata_db import get_metadata_db
from app.services.file_service import file_service
from app.services.problem_service import ProblemService
from app.services.audio_service import audio_to_text

router = APIRouter()


def generate_data():
    for number in range(1, 51):  # Generating numbers from 1 to 100
        yield f"Number: {number}\n".encode("utf-8")  # Yields data as bytes
        # Simulate a delay, can be removed or replaced with real data fetching
        time.sleep(0.1)


@router.post("/completions")
async def create_completion(
    params: CompletionParams, _: bool = Depends(verify_api_token)
):
    # Get the pre-initialized pipeline
    engine = get_model_engine()

    # select model based on params.model
    course = params.course
    formatter = format_chat_msg
    selector = generate_chat_response
    parser = local_parser

    response, reference_list = selector(
        formatter(params.messages), stream=params.stream, course=course, engine=engine
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list), media_type="text/plain"
        )
    else:
        return PlainTextResponse(response)

@router.post("/voice_completions")
async def create_voice_completion(
    params: VoiceCompletionParams, _: bool = Depends(verify_api_token)
):
    """
    Endpoint for generating voice completions.
    """
    # Get the pre-initialized pipeline
    engine = get_model_engine()
    whisper_engine = get_whisper_engine()

    # select model based on params.model
    course = params.course
    formatter = format_chat_msg
    selector = generate_chat_response
    parser = local_parser

    audio_text = audio_to_text(params.audio, whisper_engine)
    params.messages.append(Message(role="user", content=audio_text))
    response, reference_list = selector(
        formatter(params.messages), stream=params.stream, course=course, engine=engine
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list), media_type="text/plain"
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

@router.post("/practice_completion")
async def practice_completion(
    params: PracticeCompletionParams, db: Session = Depends(get_metadata_db), _: bool = Depends(verify_api_token)
):
    # check all of code_block_content problem_id file_name in params and log error if any is None
    if any(
        param is None for param in [params.problem_id, params.file_name, params.answer_content]
    ):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="problem_id, file_name, and answer_content must be provided"
        )

    metadata = file_service.get_file_metadata_by_name(db, params.file_name)
    if not metadata:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found by given name " + params.file_name
        )

    # Get problems for this file using the new relationship
    problem_content = ProblemService.get_problem_content_by_file_uuid_and_problem_id(db, str(metadata.uuid), params.problem_id)

    # Get the pre-initialized pipeline
    engine = get_model_engine()

    # select model based on params.model
    course = params.course
    formatter = format_chat_msg
    selector = generate_practice_response
    parser = local_parser

    response, reference_list = selector(
        formatter(params.messages), problem_content, params.answer_content, stream=params.stream, course=course, engine=engine, file_name=params.file_name
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list), media_type="text/plain"
        )
    else:
        return PlainTextResponse(response)