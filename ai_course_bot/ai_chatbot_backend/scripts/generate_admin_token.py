#!/usr/bin/env python3
"""
Admin Token Generator

This script generates secure random tokens for course management authentication.
Use this when setting up new environments or rotating tokens for security.
"""

import secrets
import string
import sys
from pathlib import Path


def generate_admin_token(length=64):
    """Generate a cryptographically secure random token."""
    alphabet = string.ascii_letters + string.digits + '-_'
    return ''.join(secrets.choice(alphabet) for _ in range(length))


def main():
    print("🔐 Admin Token Generator")
    print("=" * 50)

    # Generate a new token
    token = generate_admin_token()

    print(f"Generated Admin Token:")
    print(f"{token}")
    print()
    print("📋 Copy this token to your .env file:")
    print(f"admin_token={token}")
    print()
    print("⚠️  Security Reminders:")
    print("- Keep this token secure and private")
    print("- Do not commit .env files to version control")
    print("- Only share with authorized administrators")
    print("- Consider rotating tokens regularly in production")
    print()

    # Optional: Check if .env file exists and offer to update it
    env_file = Path('.env')
    if env_file.exists():
        response = input("Update existing .env file with this token? (y/N): ")
        if response.lower() == 'y':
            try:
                content = env_file.read_text()
                lines = content.split('\n')

                # Find and replace admin_token line
                updated = False
                for i, line in enumerate(lines):
                    if line.startswith('admin_token='):
                        lines[i] = f'admin_token={token}'
                        updated = True
                        break

                # If not found, add it
                if not updated:
                    lines.append(f'admin_token={token}')

                env_file.write_text('\n'.join(lines))
                print("✅ .env file updated successfully!")

            except Exception as e:
                print(f"❌ Error updating .env file: {e}")
                print("Please update manually.")
    else:
        print("💡 Create a .env file and add the admin_token line above")


if __name__ == "__main__":
    main()
