# Consolidated completions router
import time
from typing import List
from app.api.deps import verify_api_token
from app.core.models.chat_completion import (
    GeneralCompletionParams,
    FileCompletionParams,
    PracticeCompletionParams,
    PageContentParams,
    GeneratePagesParams,
    Message,
    ResponseDelta,
    TextToSpeechParams,
    VoiceTranscriptParams,
    AudioTranscript,
)
from app.dependencies.model import get_model_engine, get_whisper_engine, get_engine_for_mode
from app.services.query import top_k_selector
from app.services.generation.chat import run_chat_pipeline
from app.services.generation.tutor import run_tutor_pipeline
from app.services.generation.message_format import format_chat_msg
from app.services.request_timer import RequestTimer
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse, StreamingResponse
from sqlalchemy.orm import Session
from app.core.dbs.metadata_db import get_metadata_db
from app.services.file_service import file_service
from app.services.problem_service import ProblemService
from app.services.audio.stt import audio_to_text, audio_stream_parser
from app.services.audio.tts import (
    format_audio_text_message,
    audio_generator,
    tts_parsor,
    get_speaker_name
)
from app.services.memory.service import MemorySynopsisService

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
async def create_completion(
        params: GeneralCompletionParams | FileCompletionParams | PracticeCompletionParams,
        db: Session = Depends(get_metadata_db),
        _: bool = Depends(verify_api_token)
):
    # Create timer for tracking request latency
    timer = RequestTimer(request_id=str(time.time_ns()))
    timer.mark("request_received")

    # Dynamically select LLM mode based on tutor_mode flag
    from app.config import settings
    try:
        llm_mode = settings.get_llm_mode_for_request(params.tutor_mode)
        print(f"[INFO] Request mode: tutor_mode={params.tutor_mode}, selected LLM: {llm_mode.value}")
        llm_engine = get_engine_for_mode(llm_mode.value)
    except Exception as e:
        # If tutor mode fails and fallback is enabled, use local model
        if params.tutor_mode and settings.tutor_fallback_enabled:
            print(f"[WARNING] Failed to initialize {llm_mode.value} for tutor mode: {e}")
            print(f"[WARNING] Falling back to local model")
            llm_engine = get_engine_for_mode("local")
        else:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"LLM service unavailable: {str(e)}"
            )
    audio_text = None
    if params.audio:
        whisper_engine = get_whisper_engine()
        audio_text = audio_to_text(params.audio, whisper_engine, stream=False)
        params.messages.append(Message(role="user", content=audio_text))

    for message in params.messages:
        if message.role == "assistant":
            message.content = parse_assistant_message(message.content)
    # Select chat pipeline based on chat_type

    sid = params.sid  # Use chat_history_sid from frontend
    print(f"[INFO] Using SID: {sid}")

    print(f"[INFO] Chat Type: {type(params)}")

    problem_content = None

    # parameter check
    if isinstance(params, FileCompletionParams):
        if not params.user_focus.file_uuid:
            # Handle case where file_uuid is not provided
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="file_uuid must be provided"
            )
        print("file_uuid:", params.user_focus.file_uuid)
        print("selected_text:", params.user_focus.selected_text)
        print("chunk_index:", params.user_focus.chunk_index)
        if params.user_focus.module_uuid:
            print("module_uuid:", params.user_focus.module_uuid)
    elif isinstance(params, PracticeCompletionParams):
        problem_content = _get_problem_content(params, db)

    # Resolve module_uuid → module_path if provided
    module_path = None
    user_focus = getattr(params, 'user_focus', None)
    if user_focus and user_focus.module_uuid:
        from app.services.module_service import module_service as _module_service
        module = _module_service.get_by_uuid(db, user_focus.module_uuid)
        if not module:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Module not found: {user_focus.module_uuid}",
            )
        module_path = module.path

    # Dispatch to chat or tutor pipeline
    try:
        pipeline_fn = run_tutor_pipeline if params.tutor_mode else run_chat_pipeline
        result = await pipeline_fn(
            messages=params.messages,
            user_focus=user_focus,
            answer_content=getattr(params, 'answer_content', None),
            problem_content=problem_content,
            stream=params.stream,
            course=params.course_code,
            engine=llm_engine,
            audio_response=params.audio_response,
            sid=sid,
            timer=timer,
            audio_text=audio_text,
            module_path=module_path,
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )

    if params.stream:
        return StreamingResponse(result, media_type="text/event-stream")
    else:
        return JSONResponse(ResponseDelta(text=result).model_dump_json(exclude_unset=True))

@router.post("/page-content")
async def generate_page_content(
        params: PageContentParams,
        _: bool = Depends(verify_api_token),
):
    """Generate content for a single outline page using the local vLLM model."""
    from app.services.generation.tutor.page_content import run_page_content_pipeline

    try:
        llm_engine = get_engine_for_mode("local")
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Local LLM service unavailable: {str(e)}"
        )

    result = run_page_content_pipeline(params, llm_engine)
    return StreamingResponse(result, media_type="text/event-stream")


@router.post("/generate-pages")
async def generate_pages(
        params: GeneratePagesParams,
        _: bool = Depends(verify_api_token),
):
    """Combined pipeline: generate outline (OpenAI) + all page contents (local vLLM) in one SSE stream."""
    from app.services.generation.tutor.generate_pages import run_generate_pages_pipeline

    try:
        openai_engine = get_engine_for_mode("openai")
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"OpenAI service unavailable: {str(e)}"
        )

    try:
        local_engine = get_engine_for_mode("local")
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Local LLM service unavailable: {str(e)}"
        )

    result = run_generate_pages_pipeline(params, openai_engine, local_engine)
    return StreamingResponse(result, media_type="text/event-stream")


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
    print("finished generating audio stream")
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
    # Convert audio message to text
    if params.stream:
        # Use async streaming transcription
        return StreamingResponse(
            audio_stream_parser(params.audio, sample_rate=24000), media_type="text/event-stream"
        )
    else:
        # Use synchronous transcription
        whisper_engine = get_whisper_engine()
        transcription = audio_to_text(params.audio, whisper_engine, sample_rate=24000)
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
        # Get the pre-initialized pipeline (OpenAI client)
        engine = get_model_engine()

        # Initialize memory synopsis service
        service = MemorySynopsisService()

        # Create or update memory synopsis
        memory_synopsis_sid = await service.create_or_update_memory(sid, format_chat_msg(messages), engine)

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

def _get_problem_content(params: PracticeCompletionParams, db: Session):

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
            detail="File metadata not found by given path: " + params.file_path
        )

    problem_content = (
        ProblemService.
        get_problem_content_by_file_uuid_and_problem_id(
            db,
            str(metadata.uuid),
            params.problem_id
        )
    )

    if not problem_content:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Problem content not found for given problem_id: " + params.problem_id
                   + " and file_path: " + params.file_path
        )

    return problem_content