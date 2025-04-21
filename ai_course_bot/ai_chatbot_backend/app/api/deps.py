from typing import Optional

from ..core.security import verify_google_token
from app.config import settings
from fastapi import Header, HTTPException, status, Request


def get_current_user(authorization: str = Header(...)) -> dict:
    """
    Dependency that enforces authentication using a Google ID token.
    Expects the header in the format: "Bearer <token>".
    
    When auth_required=False in settings, accepts any token and returns a mock user.
    """
    token = authorization.replace("Bearer ", "")
    return verify_google_token(token)


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
                "email": "dev@example.com",
                "name": "Development User",
                "picture": None
            }
        return None
    token = authorization.replace("Bearer ", "")
    return verify_google_token(token)


def get_admin_user(authorization: str = Header(...)) -> dict:
    """
    Dependency that enforces admin authentication using a Google ID token.
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
        return verify_google_token(token)
    
    # If no token and auth is required, raise error
    if settings.auth_required:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required to access this endpoint"
        )
    
    # In dev mode with auth disabled, return mock user
    return {
        "user_id": "dev-user-id",
        "email": "dev@example.com",
        "name": "Development User",
        "picture": None
    }
