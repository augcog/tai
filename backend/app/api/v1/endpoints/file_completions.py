from fastapi import APIRouter, Depends, Path, HTTPException, status, Body
from pydantic import BaseModel

from ..schemas.file_completion import CompletionResponse
from ..services import completions_service
from ...deps import get_current_user

router = APIRouter()


class CompletionRequest(BaseModel):
    prompt: str


@router.post("", response_model=CompletionResponse)
def create_completion(
        fileId: str = Path(...),
        request: CompletionRequest = None,
        user: dict = Depends(get_current_user),
        rag: bool = Body(True)
):
    if not request or not request.prompt:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Missing prompt")
    result = completions_service.create_completion(fileId=fileId, prompt=request.prompt, user=user, rag=rag)
    if not result:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="File not found")
    return result
