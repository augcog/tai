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
from app.api.v1.models.files import FileRegistry  # Import to ensure table creation

# Import the new database initializer
from app.core.db_initializer import initialize_database_on_startup

from app.v1.openai_mock import router as openapi_mock_router

# from app.core.actions.model_selector import get_model

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] {%(filename)s:%(funcName)s:%(lineno)d} %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("logs.log"), logging.StreamHandler()],
)

# Initialize database with automatic file import and migration
print("🚀 Initializing database and importing existing files...")
if not initialize_database_on_startup("data"):
    print("❌ Database initialization failed! Server may not work correctly.")
    print("💡 Check the logs above for details, or run database scripts manually.")
else:
    print("✅ Database initialization completed successfully!")

# Fallback table creation (in case the initializer doesn't work)
Base.metadata.create_all(bind=engine)

# File categories are now simplified and handled automatically

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
                h2 { color: #666; margin-top: 30px; }
                .links { margin-top: 20px; }
                .links a { display: block; margin-bottom: 10px; color: #0066cc; text-decoration: none; }
                .links a:hover { text-decoration: underline; }
                .auth-section { background-color: #f5f5f5; padding: 15px; border-radius: 5px; margin-top: 20px; }
            </style>
        </head>
        <body>
            <h1>Course AI Assistant API</h1>
            <p>Welcome to the Course AI Assistant API. Use the links below to explore the API.</p>
            
            <h2>API Documentation</h2>
            <div class="links">
                <a href="/docs">API Documentation (Swagger UI)</a>
                <a href="/redoc">API Documentation (ReDoc)</a>
            </div>
            
            <h2>Testing Tools</h2>
            <div class="links">
                <a href="/file-tester">File API Tester</a>
                <a href="/database-status">Database Status</a>
                <a href="/health">Health Check</a>
            </div>
            
            <div class="auth-section">
                <h2>Authentication Testing</h2>
                <div class="links">
                    <a href="/v1/auth/test-token">Get Test Bearer Token</a>
                    <a href="/v1/auth/auth-config">View Auth Configuration</a>
                    <a href="/v1/auth/auth-status">Check Auth Status</a>
                </div>
                <p><small>Note: Test tokens are only available in development mode (auth_required=False)</small></p>
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


@app.get("/database-status")
async def database_status():
    """
    Database status endpoint - shows database initialization status
    """
    from app.core.db_initializer import get_initializer
    
    try:
        initializer = get_initializer("data")
        status = initializer.get_database_status()
        
        return {
            "status": "ok",
            "database": status,
            "message": "Database status retrieved successfully"
        }
    except Exception as e:
        return {
            "status": "error",
            "error": str(e),
            "message": "Failed to get database status"
        }


if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)
