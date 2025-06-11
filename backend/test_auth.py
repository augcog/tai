"""
Test script for authentication settings.
Run this script to verify your authentication configuration is working correctly.
"""
import os
import sys
from pathlib import Path

# Add the parent directory to sys.path to import app modules
sys.path.append(str(Path(__file__).parent))

try:
    from app.config import settings
    from app.core.security import verify_google_token
    
    print("\n=== Authentication Configuration Test ===\n")
    
    # Check if .env file exists
    env_file = Path(__file__).parent / '.env'
    if env_file.exists():
        print(f"✅ .env file found at {env_file}")
    else:
        print(f"❌ .env file not found at {env_file}")
    
    # Print current settings
    print("\nCurrent settings:")
    print(f"  auth_required: {settings.auth_required}")
    print(f"  dev_mode: {settings.dev_mode}")
    print(f"  google_client_id: {settings.google_client_id}")
    
    # Test auth bypass
    print("\nTesting auth bypass:")
    if not settings.auth_required:
        print("  ✅ Authentication is disabled (auth_required=False)")
        
        # Test getting mock user
        try:
            mock_user = verify_google_token("invalid_token")
            print(f"  ✅ Mock user retrieved: {mock_user}")
        except Exception as e:
            print(f"  ❌ Error getting mock user: {str(e)}")
    else:
        print("  ❌ Authentication is enabled (auth_required=True) - would need a valid token")
        print("  📝 To disable auth for testing, set AUTH_REQUIRED=False in .env file")
    
    print("\n=== Test Complete ===\n")
    
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure you're running this script from the correct directory.")
except Exception as e:
    print(f"Unexpected error: {e}") 