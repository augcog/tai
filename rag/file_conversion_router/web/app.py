"""FastAPI application factory for the batch upload API.

This module provides the FastAPI application for batch file upload
and conversion with SSE progress streaming.

Usage:
    # Run directly with uvicorn
    uvicorn file_conversion_router.web.app:app --reload

    # Or use the create_app factory
    from file_conversion_router.web.app import create_app
    app = create_app()
"""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from file_conversion_router.services.temp_storage_service import get_temp_storage_service

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Manage application lifecycle.

    Startup:
    - Initialize temp storage directory
    - Clean up stale job directories

    Shutdown:
    - Clean up remaining temp files (optional)
    """
    # Startup
    logger.info("Starting RAG Batch Upload API...")

    # Initialize temp storage and clean stale directories
    temp_storage = get_temp_storage_service()
    stale_cleaned = temp_storage.cleanup_stale_jobs(max_age_hours=24)
    if stale_cleaned > 0:
        logger.info(f"Cleaned {stale_cleaned} stale job directories on startup")

    stats = temp_storage.get_storage_stats()
    logger.info(f"Temp storage stats: {stats}")

    yield

    # Shutdown
    logger.info("Shutting down RAG Batch Upload API...")


def create_app(
    title: str = "RAG Batch Upload API",
    version: str = "1.0.0",
    debug: bool = False,
) -> FastAPI:
    """
    Create and configure the FastAPI application.

    Args:
        title: API title for documentation
        version: API version
        debug: Enable debug mode

    Returns:
        Configured FastAPI application
    """
    app = FastAPI(
        title=title,
        version=version,
        description="API for batch file upload and conversion with real-time progress updates",
        debug=debug,
        lifespan=lifespan,
    )

    # Configure CORS
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # Configure appropriately for production
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Import and include routers
    from file_conversion_router.web.router_batch import router as batch_router

    app.include_router(batch_router, prefix="/batch", tags=["batch"])

    # Health check endpoint
    @app.get("/health", tags=["system"])
    async def health_check():
        """Check API health status."""
        return {"status": "healthy", "version": version}

    # Storage stats endpoint
    @app.get("/storage/stats", tags=["system"])
    async def storage_stats():
        """Get temporary storage statistics."""
        temp_storage = get_temp_storage_service()
        return temp_storage.get_storage_stats()

    return app


# Create default app instance
app = create_app()


if __name__ == "__main__":
    import uvicorn

    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    uvicorn.run(
        "file_conversion_router.web.app:app",
        host="0.0.0.0",
        port=8001,
        reload=True,
    )
