from fastapi import APIRouter, Depends, Path, HTTPException, status, Body
from pydantic import BaseModel

from app.schemas.file_completion import CompletionResponse
from app.services import completions_service
from app.api.deps import verify_api_token

router = APIRouter()


class CompletionRequest(BaseModel):
    prompt: str


@router.post("", response_model=CompletionResponse)
def create_completion(
        fileId: str = Path(...),
        request: CompletionRequest = None,
        rag: bool = Body(True),
        _: bool = Depends(verify_api_token)
):
    if not request or not request.prompt:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST, detail="Missing prompt")
    result = completions_service.create_completion(
        fileId=fileId, prompt=request.prompt, rag=rag)
    if not result:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND, detail="File not found")
    return result
