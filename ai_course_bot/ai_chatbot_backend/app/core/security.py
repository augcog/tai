from fastapi import HTTPException, status
from google.auth.transport import requests as google_requests
from google.oauth2 import id_token

# Use the main application settings instead of core settings
from app.config import settings


def verify_google_token(token: str) -> dict:
    """
    Verify the Google ID token and return user information.
    Raises an HTTPException if the token is invalid.
    
    In development mode with auth_required=False, returns mock user credentials.
    """
    # Check if auth is disabled for development
    if not settings.auth_required:
        # Return mock user for development
        return {
            "user_id": "dev-user-id",
            "email": "dev@example.com",
            "name": "Development User",
            "picture": None
        }
        
    try:
        # Verify the token against Google's public keys
        idinfo = id_token.verify_oauth2_token(token, google_requests.Request(), settings.google_client_id)
        # Optionally, you can verify additional claims (e.g., hosted domain, expiry)
        return {
            "user_id": idinfo["sub"],
            "email": idinfo["email"],
            "name": idinfo.get("name"),
            "picture": idinfo.get("picture")
        }
    except ValueError:
        # Token is invalid or expired
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid Google token"
        )
