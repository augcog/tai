# Consolidated completions router
from typing import List
from app.api.deps import verify_api_token
from app.core.models.chat_completion import (
    GeneralCompletionParams,
    FileCompletionParams,
    PracticeCompletionParams,
    Message,
    ResponseDelta,
    TextToSpeechParams,
    VoiceTranscriptParams,
    AudioTranscript,
)
from app.dependencies.model import get_model_engine, get_whisper_engine
from app.services.rag_retriever import top_k_selector
from app.services.rag_generation import (
    format_chat_msg,
    generate_chat_response
)
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse, StreamingResponse
from sqlalchemy.orm import Session
from app.core.dbs.metadata_db import get_metadata_db
from app.services.file_service import file_service
from app.services.problem_service import ProblemService
from app.services.audio_service import audio_to_text, audio_stream_parser
from app.services.chat_service import (
    chat_stream_parser,
    format_audio_text_message,
    audio_generator,
    tts_parsor,
    get_speaker_name
)
from app.services.memory_synopsis_service import MemorySynopsisService
from app.services.memory_synopsis_service import MemorySynopsisServiceLong
from app.services.rag_postprocess import build_memory_synopsis, MemorySynopsis
import logging
logger = logging.getLogger(__name__)

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
    elif isinstance(params, PracticeCompletionParams):
        problem_content = _get_problem_content(params, db)

    response, reference_list = await generate_chat_response(
        params.messages,
        user_focus=getattr(params, 'user_focus', None),
        answer_content=getattr(params, 'answer_content', None),
        problem_content=problem_content,
        stream=params.stream,
        course=params.course_code,
        engine=llm_engine,
        audio_response=params.audio_response,
        sid=sid
    )
    
    create_or_update_memory_synopsis(sid, response)

    if params.stream:
        return StreamingResponse(
            chat_stream_parser(
                response,
                reference_list,
                params.audio_response,
                audio_text=audio_text,
                messages=format_chat_msg(params.messages),
                engine=llm_engine,
                old_sid=sid,
                course_code=params.course_code
            ),
            media_type="text/event-stream"
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


@router.post("/memory-synopsis")
async def create_or_update_memory_synopsis(
        sid: str,
        user_id: str, # 假设已在 GeneralCompletionParams, FileCompletionParams, PracticeCompletionParams 中添加
        messages: List[Message],
        # course_code: str,
        # _: bool = Depends(verify_api_token)
):
    """
    Create or update memory synopsis (STM) for a chat history, and then synthesize
    and update Long-Term Memory (LTM) for the user.
    """
    logger.info(f"[INFO] Memory Request received. SID: {sid}")
    try:
        # Get the pre-initialized pipeline
        engine = get_model_engine()

        # Import TOKENIZER from rag_generation
        from app.services.rag_generation import TOKENIZER

        # Initialize memory synopsis service
        stm_service = MemorySynopsisService()
        ltm_service = MemorySynopsisServiceLong()
        formatted_messages = format_chat_msg(messages)


        # 1. 步骤 a: 生成 Short-Term Memory (STM) 实例
        # 这一步是为了获取 MemorySynopsis 实例 (new_stm) 供 LTM 使用
        new_memory: MemorySynopsis = await build_memory_synopsis(
            messages=formatted_messages,
            tokenizer=TOKENIZER,
            engine=engine,
            chat_history_sid=sid # build_memory_synopsis 会自动检索旧的 STM
        )

        # 2. 步骤 b: 存储/更新 Short-Term Memory (STM)
        # 调用 STM 服务进行持久化。STM 服务内部会再次调用 build_memory_synopsis
        memory_synopsis_sid = await stm_service.create_or_update_memory(
            chat_history_sid=sid,
            messages=formatted_messages,
            engine=engine,
            tokenizer=TOKENIZER
        )

        ltm_synopsis_sid = None
        # 3. 步骤 c: 如果 STM 成功，生成并存储 Long-Term Memory (LTM)
        if memory_synopsis_sid:
            ltm_synopsis_sid = await ltm_service.create_or_update_ltm(
                user_id=user_id,
                chat_history_sid=sid,
                messages=formatted_messages,
                new_stm=new_memory, # <--- 修正：传递第一步生成的 MemorySynopsis 实例
                engine=engine,
                tokenizer=TOKENIZER
            )

        # 4. 返回结果
        if memory_synopsis_sid:
            return JSONResponse({
                "memory_synopsis_sid": memory_synopsis_sid, # STM SID
                "ltm_synopsis_sid": ltm_synopsis_sid,       # LTM SID
                "status": "success",
                "message": "Memory synopses (STM and LTM) created/updated successfully"
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
            "message": "Memory generation failed due to internal error, will retry next round"
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
