from fastapi import APIRouter, Depends, Path, HTTPException, status

from ..schemas.file import FileDetailResponse
from ..services import files_service
from ...deps import get_current_user

router = APIRouter()


@router.get("/{fileId}", response_model=FileDetailResponse)
def get_file_detail(
        fileId: str = Path(...),
        user: dict = Depends(get_current_user)
):
    file_detail = files_service.get_file_detail(fileId=fileId, user=user)
    if not file_detail:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="File not found")
    return file_detail
