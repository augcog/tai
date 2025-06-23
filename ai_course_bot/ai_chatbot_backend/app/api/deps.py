from app.config import settings
from fastapi import Header, HTTPException, status


def verify_api_token(authorization: str = Header(...)) -> bool:
    """
    Dependency that verifies the API token for NextJS <-> Backend communication.
    Expects the header in the format: "Bearer <api_token>".
    Raises 401 Unauthorized if token doesn't match the configured API token.
    """
    if not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authorization header format. Use 'Bearer <api_token>'"
        )

    token = authorization.replace("Bearer ", "")

    if token != settings.api_auth_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API token. Access denied."
        )

    return True
