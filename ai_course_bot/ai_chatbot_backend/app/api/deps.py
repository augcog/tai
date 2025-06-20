from typing import Optional

from ..core.security import verify_auth_token
from ..core.database import get_db
from app.config import settings
from fastapi import Header, HTTPException, status, Request


def get_current_user(authorization: str = Header(...)) -> dict:
    """
    Dependency that enforces authentication using a NextAuth JWT token.
    Expects the header in the format: "Bearer <token>".

    When auth_required=False in settings, accepts any token and returns a mock user.
    """
    token = authorization.replace("Bearer ", "")
    return verify_auth_token(token)


def get_current_user_optional(authorization: Optional[str] = Header(None)) -> Optional[dict]:
    """
    Optional dependency. Returns None if no token is provided.

    When auth_required=False in settings, returns a mock user even without a token.
    """
    if not authorization:
        if not settings.auth_required:
            # Return mock user without requiring token
            return {
                "user_id": "dev-user-id",
                "email": "dev@berkeley.edu",
                "name": "Development User",
                "picture": None,
                "domain": "berkeley.edu"
            }
        return None
    token = authorization.replace("Bearer ", "")
    return verify_auth_token(token)


def get_admin_user(authorization: str = Header(...)) -> dict:
    """
    Dependency that enforces admin authentication using a NextAuth JWT token.
    Expects the header in the format: "Bearer <token>".
    Raises 403 Forbidden if user is not an admin.

    When auth_required=False in settings, returns a mock admin user.
    """
    user = get_current_user(authorization)
    # In dev mode, consider all users as admins
    if not settings.auth_required:
        user['is_admin'] = True
        return user

    if not user.get("is_admin", False):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Admin access required for this endpoint"
        )
    return user


def get_admin_token_user(authorization: str = Header(...)) -> dict:
    """
    Dependency that enforces admin authentication using a static admin token.
    Expects the header in the format: "Bearer <admin_token>".
    Raises 403 Forbidden if token doesn't match the configured admin token.
    """
    if not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authorization header format. Use 'Bearer <admin_token>'"
        )

    token = authorization.replace("Bearer ", "")

    if token != settings.admin_token:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Invalid admin token. Access denied."
        )

    return {
        "user_id": "admin",
        "email": "admin@system",
        "name": "Admin User",
        "is_admin": True,
        "auth_type": "admin_token"
    }


def auth_with_query_param(request: Request, authorization: Optional[str] = Header(None)) -> dict:
    """
    A dependency that checks for authentication in either:
    1. The Authorization header
    2. An auth_token query parameter

    Useful for file endpoints where auth is needed but resources might be embedded.
    """
    # Check for auth_token query parameter
    token = request.query_params.get('auth_token')

    # If no query token, use the header
    if not token and authorization:
        token = authorization.replace("Bearer ", "")

    # If we have a token, verify it
    if token:
        return verify_auth_token(token)

    # If no token and auth is required, raise error
    if settings.auth_required:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required to access this endpoint"
        )

    # In dev mode with auth disabled, return mock user
    return {
        "user_id": "dev-user-id",
        "email": "dev@berkeley.edu",
        "name": "Development User",
        "picture": None,
        "domain": "berkeley.edu"
    }
