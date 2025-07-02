import time
from typing import Any

from app.api.deps import verify_api_token
from ai_chatbot_backend.app.core.models.chat_completion import ConversationCreateParams
from fastapi import APIRouter, Depends
from fastapi.responses import JSONResponse, StreamingResponse

from ai_chatbot_backend.app.services.practice_service import *

router = APIRouter()

@router.get("/practice/get_practice_template")
async def get_practice_template(
    params: ConversationCreateParams, _: bool = Depends(verify_api_token)
):
    """
    Get a practice template based on file name and block info.
    """
    file_name = params.file_name
    block_info = params.block_info

    template_response = generate_practice_template(file_name, block_info)

    if not template_response or template_response.body is None:
        return JSONResponse(
            status_code=404, content={"detail": "File not found or invalid parameters"}
        )
    return template_response

@router.get("/practice/get_practice_chat")
async def get_practice_chat(
    params: ConversationCreateParams, _: bool = Depends(verify_api_token)
):
    """
    Get a practice chat response based on all input parameters.
    """
    # select model based on params.model
    file_name = params.file_name
    block_info = params.block_info
    block_content = params.block_content
    question_number = params.question_number
    question_state = params.question_state
    messages = params.messages


    get_practice_response = generate_practice(file_name, block_info, block_content, messages, question_number, question_state)

    if not get_practice_response:
        return StreamingResponse(status_code=404,
            content="File not found or invalid parameters", media_type="text/plain"
        )
    return get_practice_response
