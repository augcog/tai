"""
Test script for NextAuth authentication settings.
Run this script to verify your NextAuth configuration is working correctly.
"""
import os
import sys
from pathlib import Path

# Add the parent directory to sys.path to import app modules
sys.path.append(str(Path(__file__).parent))

try:
    from app.config import settings
    from app.core.security import verify_auth_token
    
    print("\n=== NextAuth Authentication Configuration Test ===\n")
    
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
    print(f"  nextauth_secret: {'✅ Configured' if settings.nextauth_secret != 'your-nextauth-secret' else '❌ Not configured'}")
    print(f"  nextauth_url: {settings.nextauth_url}")
    print(f"  allowed_domains: {settings.allowed_domains}")
    
    # Test auth bypass
    print("\nTesting authentication:")
    if not settings.auth_required:
        print("  ✅ Authentication is disabled (auth_required=False)")
        
        # Test getting mock user
        try:
            mock_user = verify_auth_token("invalid_token")
            print(f"  ✅ Mock user retrieved: {mock_user}")
        except Exception as e:
            print(f"  ❌ Error getting mock user: {str(e)}")
    else:
        print("  ❌ Authentication is enabled (auth_required=True) - would need a valid NextAuth token")
        print("  📝 To disable auth for testing, set AUTH_REQUIRED=False in .env file")
        
        if settings.nextauth_secret == "your-nextauth-secret":
            print("  ⚠️  NextAuth secret not configured - set NEXTAUTH_SECRET in .env file")
        else:
            print("  ✅ NextAuth secret is configured")
    
    print("\n=== Configuration Recommendations ===")
    print("For development:")
    print("  AUTH_REQUIRED=false")
    print("  NEXTAUTH_SECRET=your-nextauth-secret-key")
    print("")
    print("For production:")
    print("  AUTH_REQUIRED=true")
    print("  NEXTAUTH_SECRET=<same-secret-as-frontend>")
    print("  ALLOWED_DOMAINS=berkeley.edu")
    
    print("\n=== Test Complete ===\n")
    
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure you're running this script from the correct directory.")
except Exception as e:
    print(f"Unexpected error: {e}") 