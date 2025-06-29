from fastapi import APIRouter, Request, Depends
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
import os
from pathlib import Path
from app.api.deps import verify_api_token

# Get the base directory of the application
BASE_DIR = Path(__file__).resolve().parent.parent.parent.parent.parent

# Initialize templates
templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "templates"))

router = APIRouter()


@router.get("/", response_class=HTMLResponse)
async def get_file_tester_interface(
    request: Request, _: bool = Depends(verify_api_token)
):
    """
    Serve the file tester interface HTML page.
    """
    return templates.TemplateResponse("file_tester.html", {"request": request})
