import logging
import os

import uvicorn
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

# Skip admin for now to avoid model dependencies
# from app.admin import setup_admin
from app.config import settings
from app.core.database import Base, engine

# Import the new database initializer
from app.core.db_initializer import initialize_database_on_startup

logging.basicConfig(
    level=logging.WARNING,
    format="[%(asctime)s] {%(filename)s:%(funcName)s:%(lineno)d} %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("logs.log"), logging.StreamHandler()],
)

# Initialize database with automatic file import and migration
print("🚀 Initializing database and importing existing files...")
print("📚 Course loading: from course.json (only if database is empty)")

if not initialize_database_on_startup("data"):
    print("❌ Database initialization failed! Server may not work correctly.")
    print("💡 Check the logs above for details, or run database scripts manually.")
else:
    print("✅ Database initialization completed successfully!")

# Fallback table creation (in case the initializer doesn't work)
Base.metadata.create_all(bind=engine)

print("\n🤖 Skipping AI model pipeline initialization...")
print("💡 Model-dependent endpoints may not work, but basic API will be available.")

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
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Setup templates for the file tester
templates = Jinja2Templates(directory="templates")

# Skip admin interface for now
# setup_admin(app)

# Include only basic API routes (skip model-dependent ones)
from app.api.routes import files, file_metadata, problems, courses
from fastapi import APIRouter

# Create a minimal API router with only basic endpoints
minimal_api_router = APIRouter()

# Include only the routes that don't depend on models
minimal_api_router.include_router(files.router, prefix="/files", tags=["files"])
minimal_api_router.include_router(file_metadata.router, prefix="/file-metadata", tags=["file-metadata"])
minimal_api_router.include_router(problems.router, prefix="/problems", tags=["problems"])
minimal_api_router.include_router(courses.router, prefix="/courses", tags=["courses"])

app.include_router(minimal_api_router, prefix="/api/v1")


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
            "message": "Database status retrieved successfully",
        }
    except Exception as e:
        return {
            "status": "error",
            "error": str(e),
            "message": "Failed to get database status",
        }


if __name__ == "__main__":
    # Use settings from .env file configuration
    print("🚀 Starting server...")
    print(f"📍 Environment: {settings.environment}")
    print(
        f"🔄 Auto-reload: {'disabled' if settings.is_production or not settings.RELOAD else 'enabled'}"
    )
    print(f"🌐 Host: {settings.HOST}:{settings.PORT}")
    print(f"📁 Data Directory: {settings.DATA_DIR}")

    # Determine reload setting: disabled in production or based on RELOAD setting
    reload_enabled = not settings.is_production and settings.RELOAD

    uvicorn.run(
        "main_minimal:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=reload_enabled,
        log_level="info",
    ) 