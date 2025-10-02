# Consolidated completions router
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
from app.services.file_service import file_service
from app.services.problem_service import ProblemService
from app.services.audio_service import audio_to_text, audio_stream_parser
from app.services.chat_service import chat_stream_parser, format_audio_text_message, audio_generator, tts_parsor, get_speaker_name
from app.services.memory_synopsis_service import MemorySynopsisService
import re

router = APIRouter()


def parse_assistant_message(content):
    print("Original assistant content:", content)
    # Remove THINKING-BLOCK if present
    if '<!--THINKING-BLOCK:' in content:
        thinking_end = content.find('-->')
        if thinking_end != -1:
            content = content[thinking_end + 3:].strip()

    # Remove REFERENCES if present
    references_start = content.rfind('<!--REFERENCES:')
    if references_start != -1:
        content = content[:references_start].strip()
    print("Parsed assistant content:", content)
    return content

@router.post("/completions")
async def create_text_completion(
        params: CompletionParams, _: bool = Depends(verify_api_token)
):
    # Get the pre-initialized pipeline
    llm_engine = get_model_engine()
    audio_text = None
    if params.audio:
        whisper_engine = get_whisper_engine()
        audio_text = audio_to_text(params.audio, whisper_engine, stream=False)
        params.messages.append(Message(role="user", content=audio_text))

    for message in params.messages:
        if message.role == "assistant":
            message.content = parse_assistant_message(message.content)
    # Select chat pipeline based on chat_type
    formatter = format_chat_msg

    sid = params.sid  # Use chat_history_sid from frontend
    print(f"[INFO] Using SID: {sid}")

    print(f"[INFO] Chat Type: {params.chat_type}")
    if params.chat_type == 'file':  # filechat
        if not params.user_focus.file_uuid:
            # Handle case where file_uuid is not provided
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="file_uuid must be provided"
            )
        print("file_uuid:", params.user_focus.file_uuid)
        print("selected_text:", params.user_focus.selected_text)
        print("chunk_index:", params.user_focus.chunk_index)
        response, reference_list = await generate_chat_response(
            formatter(params.messages),
            file_uuid=params.user_focus.file_uuid,
            selected_text=params.user_focus.selected_text,
            index=params.user_focus.chunk_index,
            stream=params.stream,
            course=params.course_code,
            engine=llm_engine,
            audio_response=params.audio_response,
            sid=sid
        )
    else:   # general case
        response, reference_list = await generate_chat_response(
            formatter(params.messages),
            stream=params.stream,
            course=params.course_code,
            engine=llm_engine,
            audio_response=params.audio_response,
            sid=sid
        )

    parser = chat_stream_parser

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list, params.audio_response,audio_text=audio_text, messages=formatter(params.messages), engine=llm_engine, old_sid=sid,course_code=params.course_code), media_type="text/event-stream"
        )
    else:
        return JSONResponse(ResponseDelta(text=response).model_dump_json(exclude_unset=True))

@router.post("/text_completions")
async def create_text_completion(
        params: TextCompletionParams, _: bool = Depends(verify_api_token)
):
    # Get the pre-initialized pipeline
    engine = get_model_engine()

    # Select chat pipeline based on chat_type
    course = params.course
    formatter = format_chat_msg

    sid = params.sid  # Use chat_history_sid from frontend
    print(f"[INFO] Using SID: {sid}")

    print(f"[INFO] Chat Type: {params.chat_type}")
    if params.chat_type == 'file':  # filechat
        if not params.file_uuid:
            # Handle case where file_uuid is not provided
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="file_uuid must be provided"
            )
        response, reference_list = await generate_chat_response(
            formatter(params.messages),
            file_uuid=params.file_uuid,
            selected_text=params.selected_text,
            index=params.index,
            stream=params.stream,
            course=course,
            engine=engine,
            audio_response=params.audio_response,
            sid=sid
        )
    else:   # general case
        response, reference_list = await generate_chat_response(
            formatter(params.messages),
            stream=params.stream,
            course=course,
            engine=engine,
            audio_response=params.audio_response,
            sid=sid
        )

    parser = chat_stream_parser

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list, params.audio_response, messages=formatter(params.messages), engine=engine, old_sid=sid), media_type="text/event-stream"
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
        formatter(params.messages), stream=params.stream, course=course, engine=engine,audio_response=params.audio_response, sid=params.sid
    )

    if params.stream:
        return StreamingResponse(
            parser(response, reference_list, params.audio_response,audio_text=audio_text, course_code=course), media_type="text/event-stream"
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

    print("tts")
    print("text:", params.text)
    print("class_code:", params.class_code)

    answer = parse_assistant_message(params.text)

    audio_message = format_audio_text_message(answer)
    print("answer:", answer)
    speaker_name= get_speaker_name(params.class_code)
    stream = audio_generator(audio_message, stream=params.stream, speaker_name=speaker_name)

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
        transcription = audio_to_text(
            params.audio, whisper_engine, stream=params.stream, sample_rate=24000)
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
    # check all of code_block_content problem_id file_path in params and log error if any is None
    if any(
            param is None for param in [params.problem_id, params.file_path, params.answer_content]
    ):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="problem_id, file_path, and answer_content must be provided"
        )

    metadata = file_service.get_file_metadata_by_path(db, params.file_path)
    if not metadata:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="File metadata not found by given path " + params.file_path
        )

    # Get problems for this file using the new relationship
    problem_content = ProblemService.get_problem_content_by_file_uuid_and_problem_id(db, str(metadata.uuid),
                                                                                     params.problem_id)
    formatter = format_chat_msg
    sid = sid_from_history(formatter(params.messages))
    print(f"[INFO] Generated SID: {sid}")
    # Get the pre-initialized pipeline
    llm_engine = get_model_engine()

    # select model based on params.model
    course = params.course
    formatter = format_chat_msg
    selector = generate_practice_response
    parser = chat_stream_parser


    response, reference_list = selector(
        formatter(params.messages), problem_content, params.answer_content, stream=params.stream, course=course,
        engine=llm_engine
    )


    if params.stream:
        return StreamingResponse(
            parser(response, reference_list, messages=formatter(params.messages), engine=llm_engine, old_sid=sid), media_type="text/event-stream"
        )
    else:
       return JSONResponse(ResponseDelta(text=response).model_dump_json(exclude_unset=True))


@router.post("/memory-synopsis")
async def create_or_update_memory_synopsis(
        sid: str,
        messages: List[Message],
        # course_code: str,
        # _: bool = Depends(verify_api_token)
):
    """
    Create or update memory synopsis for a chat history.

    Args:
        sid: chat_history_sid from frontend
        messages: List of chat messages
        course_code: Course code for context

    Returns:
        JSON response with memory_synopsis_sid if successful, error message if failed
    """
    try:
        # Get the pre-initialized pipeline
        engine = get_model_engine()

        # Import TOKENIZER from rag_generation
        from app.services.rag_generation import TOKENIZER

        # Initialize memory synopsis service
        service = MemorySynopsisService()

        # Create or update memory synopsis
        memory_synopsis_sid = await service.create_or_update_memory(sid, format_chat_msg(messages), engine, TOKENIZER)

        if memory_synopsis_sid:
            return JSONResponse({
                "memory_synopsis_sid": memory_synopsis_sid,
                "status": "success",
                "message": "Memory synopsis created/updated successfully"
            })
        else:
            return JSONResponse({
                "status": "failed",
                "message": "Memory generation failed, will retry next round"
            })

    except Exception as e:
        print(f"[INFO] Memory synopsis endpoint failed: {e}")
        return JSONResponse({
            "status": "failed",
            "message": "Memory generation failed, will retry next round"
        })
