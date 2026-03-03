"""Web layer for the file conversion router.

This package provides FastAPI endpoints for batch file upload and conversion
with real-time progress updates via Server-Sent Events (SSE).

Main components:
- app.py: FastAPI application factory
- router_batch.py: Batch upload API endpoints
- schemas.py: Pydantic models for request/response validation
"""

from file_conversion_router.web.app import create_app, app

__all__ = ["create_app", "app"]
