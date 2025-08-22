from app.config import settings
from fastapi import HTTPException, status, Depends, Query
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

security = HTTPBearer()

def verify_api_token(credentials: HTTPAuthorizationCredentials = Depends(security)) -> bool:
    """
    Dependency that verifies the API token for NextJS <-> Backend communication.
    Expects the header in the format: "Bearer <api_token>".
    Raises 401 Unauthorized if token doesn't match the configured API token.
    """
    if credentials.credentials != settings.api_auth_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API token. Access denied.",
        )

    return True

def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)) -> dict:
    """
    Dependency that returns the current user for authenticated requests.
    This is a simple implementation that just validates the token.
    """
    if credentials.credentials != settings.api_auth_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API token. Access denied.",
        )
    
    return {"user": "api_user", "token": credentials.credentials}

def get_admin_user(credentials: HTTPAuthorizationCredentials = Depends(security)) -> dict:
    """
    Dependency that returns the admin user for authenticated requests.
    This validates against the admin token.
    """
    if credentials.credentials != settings.admin_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid admin token. Access denied.",
        )
    
    return {"user": "admin", "token": credentials.credentials}

def auth_with_query_param(token: str = Query(...)) -> dict:
    """
    Dependency that validates token from query parameter.
    """
    if token != settings.api_auth_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API token. Access denied.",
        )
    
    return {"user": "api_user", "token": token}
