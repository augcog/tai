from fastapi import HTTPException, status
from google.auth.transport import requests as google_requests
from google.oauth2 import id_token

# TODO: Replace with TAI's Google Client ID
GOOGLE_CLIENT_ID = "your-google-client-id"


def verify_google_token(token: str) -> dict:
    """
    Verify the Google ID token and return user information.
    Raises an HTTPException if the token is invalid.
    """
    try:
        # Verify the token against Google's public keys
        idinfo = id_token.verify_oauth2_token(token, google_requests.Request(), GOOGLE_CLIENT_ID)
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
