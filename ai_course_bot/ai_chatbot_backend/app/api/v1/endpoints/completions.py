import os

from fastapi import APIRouter, Depends, Query
from fastapi.responses import StreamingResponse

from app.api.v1.utils.stream_processing import extract_text_and_references
from app.api.v1.schemas.completion import CompletionCreateParams
from app.api.v1.services.rag_selector import rag_json_stream_generator, format_chat_msg
from app.core.actions.model_selector import course_selection
from app.dependencies.model import get_model_pipeline

router = APIRouter()


async def process_completion(params: CompletionCreateParams, pipeline):
    course = params.course
    rag = params.rag
    print(f"Processing completion for course: {course} (RAG={'enabled' if rag else 'disabled'})")

    # Select model based on params.course if needed.
    course_model_address = course_selection.get(course, "default")

    # Prepare helper functions
    formatter = format_chat_msg
    selector = rag_json_stream_generator

    # Use the embedding directory from an environment variable (if set) or fall back to the default.
    embedding_dir = os.environ.get("EMBEDDING_DIR",
                                   "/home/bot/localgpt/tai/ai_course_bot/ai-chatbot-backend/app/embedding/")
    # Call the selector with the appropriate RAG flag.
    response = selector(formatter(params.messages), stream=params.stream, course=course, pipeline=pipeline, rag=rag,
                        embedding_dir=embedding_dir)

    if params.stream:
        # Return the JSON stream directly with the appropriate media type.
        return StreamingResponse(response, media_type="application/json")
    else:
        # In non-streaming mode, aggregate all tokens and extract references.
        full_text, references = extract_text_and_references(response)
        return {
            "content": full_text,
            "references": references,  # This could be an empty list if no references are available.
        }

@router.post("")
async def create_completion(
        params: CompletionCreateParams,
        pipeline = Depends(get_model_pipeline),
):
    return await process_completion(params, pipeline)