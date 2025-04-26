import logging
import os

import uvicorn
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from app.admin import setup_admin
from app.api.v1.router import router as v1_router
from app.core.database import engine
from app.core.models.courses import Base
from app.v1.openai_mock import router as openapi_mock_router

# from app.core.actions.model_selector import get_model

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] {%(filename)s:%(funcName)s:%(lineno)d} %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("logs.log"), logging.StreamHandler()],
)

# Create the database tables
Base.metadata.create_all(bind=engine)

app = FastAPI(
    title="Course AI Assistant API",
    description="API for interacting with the course AI assistant and file management",
    version="0.1.0",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
)

# Add CORS middleware to allow the frontend to access the API
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Update with your frontend origins in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount static files directory for serving test files
curr_abs_path = os.path.dirname(os.path.abspath(__file__))
static_dir = os.path.join(curr_abs_path, "static")
# app.mount("/static", StaticFiles(directory="static"), name="static")
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Setup templates for the file tester
templates = Jinja2Templates(directory="templates")

# Setup the admin interface
setup_admin(app)

# Include routers
app.include_router(openapi_mock_router)
app.include_router(v1_router, prefix="/v1", tags=["v1"])


@app.get("/", response_class=HTMLResponse)
async def root():
    return RedirectResponse(url="/home")


@app.get("/home", response_class=HTMLResponse)
async def home():
    return """
    <html>
        <head>
            <title>Course AI Assistant API</title>
            <style>
                body { font-family: Arial, sans-serif; max-width: 800px; margin: 0 auto; padding: 20px; }
                h1 { color: #333; }
                .links { margin-top: 20px; }
                .links a { display: block; margin-bottom: 10px; color: #0066cc; text-decoration: none; }
                .links a:hover { text-decoration: underline; }
            </style>
        </head>
        <body>
            <h1>Course AI Assistant API</h1>
            <p>Welcome to the Course AI Assistant API. Use the links below to explore the API.</p>
            <div class="links">
                <a href="/docs">API Documentation (Swagger UI)</a>
                <a href="/redoc">API Documentation (ReDoc)</a>
                <a href="/file-tester">File API Tester</a>
            </div>
        </body>
    </html>
    """


@app.get("/file-tester", response_class=HTMLResponse)
async def file_tester(request: Request):
    """
    Simple HTML interface for testing the Local File Retrieval API
    """
    return templates.TemplateResponse("file_tester.html", {"request": request})


@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "ok", "version": app.version}


if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)
