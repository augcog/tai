from fastapi import HTTPException, status, Depends, Header
import jwt
import logging
from typing import Optional, Dict, Any
import time
import uuid

# Use the main application settings instead of core settings
from app.config import settings

logger = logging.getLogger(__name__)


def verify_nextauth_token(token: str) -> dict:
    """
    Verify NextAuth JWT token using the NextAuth secret.
    This is the primary authentication method for the API.
    """
    try:
        # Remove 'Bearer ' prefix if present
        clean_token = token.replace('Bearer ', '') if token.startswith('Bearer ') else token
        
        if not settings.nextauth_secret or settings.nextauth_secret == "your-nextauth-secret":
            raise ValueError("NextAuth secret not configured")
        
        # Decode the NextAuth JWT token
        decoded = jwt.decode(
            clean_token, 
            settings.nextauth_secret, 
            algorithms=["HS256"],
            options={"verify_exp": True}
        )
        
        # Validate domain if required
        email = decoded.get("email")
        if email and settings.allowed_domains:
            domain = email.split('@')[1] if '@' in email else None
            if domain not in settings.allowed_domains:
                raise ValueError(f"Email domain {domain} not allowed. Allowed domains: {settings.allowed_domains}")
        
        return {
            "user_id": decoded.get("sub") or decoded.get("email"),
            "email": decoded.get("email"), 
            "name": decoded.get("name"),
            "picture": decoded.get("picture") or decoded.get("image"),
            "domain": email.split('@')[1] if email and '@' in email else None
        }
        
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired"
        )
    except jwt.InvalidTokenError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=f"Invalid token: {str(e)}"
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Token verification failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=f"Authentication failed: {str(e)}"
        )


def verify_auth_token(token: str) -> dict:
    """
    Main token verification function.
    In development mode, returns mock user. In production, verifies NextAuth token.
    """
    # Check if auth is disabled for development
    if not settings.auth_required:
        # Return mock user for development
        return {
            "user_id": "dev-user-id",
            "email": "dev@berkeley.edu",
            "name": "Development User",
            "picture": None,
            "domain": "berkeley.edu"
        }
    
    # Production mode - verify NextAuth token
    return verify_nextauth_token(token)


def get_current_user(authorization: Optional[str] = Header(None)) -> Dict[str, Any]:
    """
    FastAPI dependency to get the current authenticated user from the Authorization header.
    """
    if not authorization:
        if not settings.auth_required:
            # Return mock user in development mode
            return verify_auth_token("")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header is required"
        )
    
    if not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authorization header format. Use 'Bearer <token>'"
        )
    
    token = authorization.split(" ")[1]
    return verify_auth_token(token)


def generate_test_token() -> Dict[str, Any]:
    """
    Generate a test token for development purposes.
    This creates a mock NextAuth-style JWT token that can be used for testing.
    """
    if settings.auth_required:
        return {
            "error": "Test tokens can only be generated in development mode",
            "message": "Set auth_required=False in your configuration to use test tokens"
        }
    
    # Create a mock NextAuth-style JWT token for testing
    payload = {
        "sub": "test-user-" + str(uuid.uuid4())[:8],
        "email": "test@berkeley.edu",
        "name": "Test User",
        "picture": None,
        "iat": int(time.time()),
        "exp": int(time.time()) + 3600,  # 1 hour expiry
        "iss": "nextauth"
    }
    
    # Use the NextAuth secret if available, otherwise use a test secret
    secret = settings.nextauth_secret if settings.nextauth_secret != "your-nextauth-secret" else "test-secret-key"
    test_token = jwt.encode(payload, secret, algorithm="HS256")
    
    return {
        "access_token": test_token,
        "token_type": "bearer",
        "expires_in": 3600,
        "user": {
            "user_id": payload["sub"],
            "email": payload["email"],
            "name": payload["name"]
        },
        "note": "This is a test token for development only. It mimics NextAuth JWT structure."
    }
