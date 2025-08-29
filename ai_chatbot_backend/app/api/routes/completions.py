# Consolidated completions router
import time

from app.api.deps import verify_api_token
from app.core.models.chat_completion import *
from app.dependencies.model import get_model_engine, get_whisper_engine
from app.services.rag_retriever import top_k_selector
from app.services.rag_generation import (
    format_chat_msg,
    generate_chat_response,
    generate_practice_response,
    local_parser,
)
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse, PlainTextResponse, StreamingResponse
from sqlalchemy.orm import Session
from app.core.dbs.metadata_db import get_metadata_db
from app.core.dbs.content_db import get_content_db
from app.services.file_service import file_service
from app.services.problem_service import ProblemService
from app.services.audio_service import audio_to_text, audio_stream_parser
from app.services.chat_service import chat_stream_parser, format_audio_text_message, audio_generator, tts_parsor

router = APIRouter()


def generate_data():
    for number in range(1, 51):  # Generating numbers from 1 to 100
        yield f"Number: {number}\n".encode("utf-8")  # Yields data as bytes
        # Simulate a delay, can be removed or replaced with real data fetching
        time.sleep(0.1)


import json, base64, hashlib, secrets
def sid_from_history(messages):
    hist = [{"r": getattr(m, "role", "user"),
             "c": (getattr(m, "content", "") or "").split("<|begin_of_reference|>", 1)[0]}
            for m in messages[:-1]]
    # print(f"[INFO] History for SID generation: {hist}")
    digest = hashlib.blake2b(
        json.dumps(hist, separators=(",", ":"), ensure_ascii=False).encode(),
        digest_size=12
    ).digest()
    return base64.urlsafe_b64encode(digest).decode().rstrip("=")


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

    sid = sid_from_history(formatter(params.messages))
    print(f"[INFO] Generated SID: {sid}")

    response, reference_list = await selector(
        formatter(params.messages), stream=params.stream, course=course, engine=engine, audio_response=params.audio_response, sid=sid
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list, messages=formatter(params.messages), engine=engine, old_sid=sid), media_type="text/plain"
        )
    else:
        return PlainTextResponse(response)


@router.post("/text_completions")
async def create_text_completion(
        params: CompletionParams, _: bool = Depends(verify_api_token)
):
    # Get the pre-initialized pipeline
    engine = get_model_engine()

    # select model based on params.model
    course = params.course
    formatter = format_chat_msg
    selector = generate_chat_response
    parser = chat_stream_parser

    response, reference_list = await selector(
        formatter(params.messages), stream=params.stream, course=course, engine=engine, audio_response=params.audio_response
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list, params.audio_response), media_type="text/event-stream"
        )
    else:
        return JSONResponse(ResponseDelta(text=response).model_dump_json(exclude_unset=True))


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
    parser = chat_stream_parser

    audio_text = audio_to_text(params.audio, whisper_engine, stream=False)
    params.messages.append(Message(role="user", content=audio_text))
    response, reference_list = await selector(
        formatter(params.messages), stream=params.stream, course=course, engine=engine
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list,params.audio_response), media_type="text/event-stream"
        )
    else:
        return JSONResponse(ResponseDelta(text=response).model_dump_json(exclude_unset=True))

@router.post("/tts")
async def text_to_speech(
        params: TextToSpeechParams, _: bool = Depends(verify_api_token)
):
    """
    Endpoint for converting text to speech.
    """
    # Convert text message to audio
    audio_message = format_audio_text_message(params.text)
    stream = await audio_generator(audio_message, stream=params.stream)

    if params.stream:
        return StreamingResponse(
            tts_parsor(stream), media_type="text/event-stream"
        )
    else:
        return None

@router.post("/voice_to_text")
async def voice_to_text(
        params: VoiceTranscriptParams, _: bool = Depends(verify_api_token)
):
    """
    Endpoint for converting voice messages to text.
    """
    # Get the pre-initialized Whisper model engine
    whisper_engine = get_whisper_engine()

    # Convert audio message to text
    if params.stream:
        stream = await audio_to_text(params.audio, whisper_engine, stream=params.stream, sample_rate=24000)
        return StreamingResponse(
            audio_stream_parser(stream), media_type="text/event-stream"
        )
    else:
        transcription = audio_to_text(params.audio, whisper_engine, stream=params.stream, sample_rate=24000)
        return JSONResponse(AudioTranscript(text=transcription).model_dump_json(exclude_unset=True))


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
            param is None for param in [params.problem_id, params.file_path, params.answer_content]
    ):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="problem_id, file_name, and answer_content must be provided"
        )

    metadata = file_service.get_file_metadata_by_name(db, params.file_path)
    if not metadata:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found by given name " + params.file_path
        )

    # Get problems for this file using the new relationship
    problem_content = ProblemService.get_problem_content_by_file_uuid_and_problem_id(db, str(metadata.uuid),
                                                                                     params.problem_id)

    # Get the pre-initialized pipeline
    engine = get_model_engine()

    # select model based on params.model
    course = params.course
    formatter = format_chat_msg
    selector = generate_practice_response
    parser = local_parser

    response, reference_list = selector(
        formatter(params.messages), problem_content, params.answer_content, stream=params.stream, course=course,
        engine=engine
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list), media_type="text/plain"
        )
    else:
        return PlainTextResponse(response)

@router.post("/practice_completion_v2")
async def practice_completion_v2(
        params: PracticeCompletionParams, db: Session = Depends(get_content_db), _: bool = Depends(verify_api_token)
):
    # check all of code_block_content problem_id file_path in params and log error if any is None
    if any(
            param is None for param in [params.problem_id, params.file_path, params.answer_content]
    ):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="problem_id, file_path, and answer_content must be provided"
        )
    # for new api, front end passes in file path instead of file name
    metadata = file_service.get_file_metadata_by_file_path(db, params.file_path)
    if not metadata:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found by given name " + params.file_path
        )

    # Get problems for this file using the new relationship
    problem_content = ProblemService.get_problem_content_by_file_uuid_and_problem_id(db, str(metadata.uuid),
                                                                                     params.problem_id)

    # Get the pre-initialized pipeline
    engine = get_model_engine()

    # select model based on params.model
    course = params.course
    formatter = format_chat_msg
    selector = generate_practice_response
    parser = local_parser

    response, reference_list = selector(
        formatter(params.messages), problem_content, params.answer_content, stream=params.stream, course=course,
        engine=engine
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list), media_type="text/plain"
        )
    else:
        return PlainTextResponse(response)
