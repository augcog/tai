from typing import Optional

from ..core.security import verify_google_token
from fastapi import Header


def get_current_user(authorization: str = Header(...)) -> dict:
    """
    Dependency that enforces authentication using a Google ID token.
    Expects the header in the format: "Bearer <token>".
    """
    token = authorization.replace("Bearer ", "")
    return verify_google_token(token)


def get_current_user_optional(authorization: Optional[str] = Header(None)) -> Optional[dict]:
    """
    Optional dependency. Returns None if no token is provided.
    """
    if not authorization:
        return None
    token = authorization.replace("Bearer ", "")
    return verify_google_token(token)
