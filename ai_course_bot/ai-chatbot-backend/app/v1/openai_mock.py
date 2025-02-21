from app.core.actions.llama_seletor import local_selector, local_parser, local_formatter
from app.core.actions.model_selector import course_selection
from app.core.models.chat_completion import CompletionCreateParams
from app.dependencies.model import get_model_pipeline
from fastapi import APIRouter, Depends
from fastapi.responses import StreamingResponse

router = APIRouter(prefix="/api/chat")


async def process_completion(params: CompletionCreateParams, pipeline, rag: bool):
    course = params.course
    print(f"Processing completion for course: {course} (RAG={'enabled' if rag else 'disabled'})")

    # Select model based on params.course if needed.
    course_model_address = course_selection.get(course, "default")

    # Prepare helper functions
    formatter = local_formatter
    selector = local_selector
    parser = local_parser

    # Call the selector with the appropriate RAG flag.
    response = selector(formatter(params.messages), stream=params.stream, course=course, pipeline=pipeline, rag=rag)

    if params.stream:
        return StreamingResponse(parser(response), media_type="text/plain")
    else:
        # TODO: Revisit the code here
        return {"message": f"This is a test response with{' RAG' if rag else 'out RAG'}."}


@router.post("/completions")
async def create_completion(
        params: CompletionCreateParams,
        pipeline=Depends(get_model_pipeline)
):
    return await process_completion(params, pipeline, rag=True)


@router.post("/completions/no-rag")
async def create_completion_no_rag(
        params: CompletionCreateParams,
        pipeline=Depends(get_model_pipeline)
):
    return await process_completion(params, pipeline, rag=False)
