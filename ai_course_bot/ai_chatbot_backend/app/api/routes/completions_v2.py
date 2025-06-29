import os
import json

from fastapi import APIRouter, Depends
from fastapi.responses import StreamingResponse

from app.schemas.completion import CompletionCreateParams, ChatCompletionChunk, ToolCall
from app.services.rag_selector import rag_json_stream_generator, format_chat_msg
from app.utils.stream_processing import openai_format_stream, extract_text_and_references_from_openai_format
from app.core.actions.model_selector import course_selection
from app.dependencies.model import get_model_engine
from app.api.deps import verify_api_token

router = APIRouter()


async def process_completion(params: CompletionCreateParams):
    course = params.course
    rag = params.rag
    print(
        f"Processing completion for course: {course} (RAG={'enabled' if rag else 'disabled'})")

    # Get the pre-initialized pipeline
    pipeline = get_model_engine()

    # Select model based on params.course if needed.
    course_model_address = course_selection.get(course, "default")

    # Prepare helper functions
    formatter = format_chat_msg
    selector = rag_json_stream_generator

    # Use the embedding directory from an environment variable (if set) or fall back to the default.
    embedding_dir = os.environ.get("EMBEDDING_DIR",
                                   "/home/bot/localgpt/tai/ai_course_bot/ai_chatbot_backend/app/embedding/")
    # Call the selector with the appropriate RAG flag.
    response = selector(formatter(params.messages), stream=params.stream, course=course, pipeline=pipeline, rag=rag,
                        embedding_dir=embedding_dir)

    if params.stream:
        # Convert to OpenAI format
        openai_stream = openai_format_stream(response)
        return StreamingResponse(openai_stream, media_type="application/json")
    else:
        # Use OpenAI format and convert to single response
        openai_stream = openai_format_stream(response)
        full_text, references = extract_text_and_references_from_openai_format(
            openai_stream)

        # Create tool calls for references
        tool_calls = None
        if references:
            tool_calls = [
                ChatCompletionChunk.create_reference_tool_call(
                    f"Reference {i+1}",
                    url
                ) for i, url in enumerate(references) if url
            ]

        # Create a single chunk response with all content
        chunk = ChatCompletionChunk.create_chunk(
            role="assistant",
            content=full_text,
            finish_reason="stop",
            tool_calls=tool_calls
        )
        return chunk.model_dump()


@router.post("/completions_v2")
async def create_completion(params: CompletionCreateParams, _: bool = Depends(verify_api_token)):
    """OpenAI-compatible chat completions endpoint

    OpenAI's Relevant Doc:
        https://platform.openai.com/docs/api-reference/chat-streaming/streaming
    """
    return await process_completion(params)
