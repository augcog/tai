"""
Authentication endpoints for the Course AI Assistant API.

Provides endpoints for:
- Token validation
- Test token generation (development only)
- User profile information
- Authentication status checks
"""

from fastapi import APIRouter, Depends, HTTPException, status, Header
from typing import Optional, Dict, Any
import logging

from app.core.security import get_current_user, verify_auth_token, generate_test_token
from app.config import settings

router = APIRouter()
logger = logging.getLogger(__name__)


@router.get("/validate-token")
async def validate_token(current_user: Dict[str, Any] = Depends(get_current_user)):
    """
    Validate the current authentication token and return user information.
    
    This endpoint can be used by frontend applications to:
    - Verify if a token is still valid
    - Get current user information
    - Check authentication status
    """
    return {
        "valid": True,
        "user": current_user,
        "message": "Token is valid"
    }


@router.get("/me")
async def get_current_user_profile(current_user: Dict[str, Any] = Depends(get_current_user)):
    """
    Get the current authenticated user's profile information.
    """
    return {
        "user": current_user,
        "authenticated": True
    }


@router.get("/test-token")
async def get_test_token():
    """
    Generate a test bearer token for development and testing purposes.
    
    This endpoint is only available when auth_required=False (development mode).
    The generated token can be used to test authenticated endpoints.
    
    Returns:
        - access_token: The bearer token to use in Authorization headers
        - token_type: Always "bearer"
        - expires_in: Token expiration time in seconds
        - user: Mock user information associated with the token
    """
    result = generate_test_token()
    
    if "error" in result:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=result["error"] + ". " + result.get("message", "")
        )
    
    return result


@router.post("/verify-token")
async def verify_token_endpoint(authorization: Optional[str] = Header(None)):
    """
    Verify a token provided in the Authorization header or request body.
    
    This is useful for frontend applications to validate tokens before making requests.
    """
    if not authorization:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Authorization header is required"
        )
    
    if not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid authorization header format. Use 'Bearer <token>'"
        )
    
    token = authorization.split(" ")[1]
    
    try:
        user_info = verify_auth_token(token)
        return {
            "valid": True,
            "user": user_info,
            "message": "Token verification successful"
        }
    except HTTPException as e:
        return {
            "valid": False,
            "error": e.detail,
            "message": "Token verification failed"
        }


@router.get("/auth-config")
async def get_auth_config():
    """
    Get the current authentication configuration.
    
    This endpoint provides information about:
    - Whether authentication is required
    - Allowed domains for registration
    - NextAuth configuration status
    """
    return {
        "auth_required": settings.auth_required,
        "allowed_domains": settings.allowed_domains,
        "nextauth_enabled": bool(settings.nextauth_secret and settings.nextauth_secret != "your-nextauth-secret"),
        "nextauth_url": settings.nextauth_url,
        "development_mode": not settings.auth_required,
        "auth_method": "NextAuth JWT tokens only",
        "message": "Authentication configuration retrieved successfully"
    }


@router.get("/auth-status")
async def get_auth_status(authorization: Optional[str] = Header(None)):
    """
    Check authentication status without requiring authentication.
    
    This endpoint returns the authentication status and can be called
    without providing credentials.
    """
    if not authorization:
        return {
            "authenticated": False,
            "auth_required": settings.auth_required,
            "message": "No authorization header provided"
        }
    
    try:
        if authorization.startswith("Bearer "):
            token = authorization.split(" ")[1]
            user_info = verify_auth_token(token)
            return {
                "authenticated": True,
                "user": user_info,
                "auth_required": settings.auth_required,
                "message": "User is authenticated"
            }
        else:
            return {
                "authenticated": False,
                "auth_required": settings.auth_required,
                "error": "Invalid authorization header format",
                "message": "Authorization header must start with 'Bearer '"
            }
    except Exception as e:
        return {
            "authenticated": False,
            "auth_required": settings.auth_required,
            "error": str(e),
            "message": "Authentication failed"
        } 