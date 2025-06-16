# Authentication Setup Guide

This guide explains how to set up and use the authentication system for the Course AI Assistant API.

## Overview

The API uses NextAuth JWT token authentication with two modes:

1. **Development Mode** (`auth_required=false`): Uses mock authentication for testing
2. **Production Mode** (`auth_required=true`): Verifies NextAuth JWT tokens from your frontend

## Authentication Flow

Your frontend handles the complete authentication flow:
1. User signs in with Google OAuth via NextAuth.js
2. NextAuth.js creates a JWT token using your NextAuth secret
3. Frontend sends this JWT token to the backend API
4. Backend verifies the JWT token using the same NextAuth secret

## Quick Start (Development Mode)

For development and testing, you can disable authentication:

1. Create a `.env` file in the project root:
```env
AUTH_REQUIRED=false
DEV_MODE=true
NEXTAUTH_SECRET=your-nextauth-secret-key
```

2. Start the server:
```bash
python main.py
```

3. Get a test token:
```bash
curl http://localhost:8000/v1/auth/test-token
```

4. Use the token in requests:
```bash
curl -H "Authorization: Bearer <test-token>" http://localhost:8000/v1/auth/me
```

## Production Setup

### Environment Configuration

Create a `.env` file with your NextAuth configuration:

```env
# Production Authentication
AUTH_REQUIRED=true
DEV_MODE=false

# NextAuth Configuration (must match your frontend)
NEXTAUTH_SECRET=your-nextauth-secret-key
NEXTAUTH_URL=http://localhost:3000

# Domain Restrictions
ALLOWED_DOMAINS=berkeley.edu
```

**Important**: The `NEXTAUTH_SECRET` must match exactly with the secret used in your NextAuth.js frontend configuration.

### Frontend Integration

Your frontend is already set up with NextAuth.js and Google OAuth. The backend will automatically validate the JWT tokens that NextAuth.js generates.

When making API calls from your frontend, include the NextAuth session token:

```javascript
import { getSession } from 'next-auth/react';

async function callAPI() {
  const session = await getSession();
  
  const response = await fetch('/api/backend/v1/files', {
    headers: {
      'Authorization': `Bearer ${session.accessToken}`
    }
  });
  
  return response.json();
}
```

## API Endpoints

### Authentication Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/v1/auth/test-token` | GET | Generate test token (dev mode only) |
| `/v1/auth/validate-token` | GET | Validate current token |
| `/v1/auth/me` | GET | Get current user profile |
| `/v1/auth/verify-token` | POST | Verify a specific token |
| `/v1/auth/auth-config` | GET | Get authentication configuration |
| `/v1/auth/auth-status` | GET | Check authentication status |

### Using Authentication in Other Endpoints

Most API endpoints require authentication. Include the token in the Authorization header:

```bash
curl -H "Authorization: Bearer <your-token>" \
     http://localhost:8000/v1/files
```

## Testing Authentication

### 1. Check Authentication Status

```bash
curl http://localhost:8000/v1/auth/auth-status
```

### 2. Get Test Token (Development Only)

```bash
curl http://localhost:8000/v1/auth/test-token
```

Response:
```json
{
  "access_token": "eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9...",
  "token_type": "bearer",
  "expires_in": 3600,
  "user": {
    "user_id": "test-user-12345678",
    "email": "test@berkeley.edu",
    "name": "Test User"
  },
  "note": "This is a test token for development only."
}
```

### 3. Validate Token

```bash
curl -H "Authorization: Bearer <token>" \
     http://localhost:8000/v1/auth/validate-token
```

## Domain Restrictions

The system can restrict access to specific email domains (e.g., berkeley.edu):

```env
ALLOWED_DOMAINS=berkeley.edu,example.edu
```

Users with emails from other domains will be rejected during authentication.

## Error Handling

Common authentication errors:

| Error Code | Description | Solution |
|------------|-------------|----------|
| 401 | Invalid or expired token | Get a new token |
| 403 | Domain not allowed | Use allowed email domain |
| 400 | Invalid token format | Use "Bearer <token>" format |

## Security Considerations

1. **Never expose credentials**: Keep your `.env` file secure and never commit it to version control
2. **Use HTTPS in production**: Always use HTTPS for production deployments
3. **Token expiration**: Implement token refresh in your frontend
4. **Domain restrictions**: Always restrict to your organization's domains in production

## Troubleshooting

### Common Issues

1. **"Google Client ID not configured"**
   - Set `GOOGLE_CLIENT_ID` in your `.env` file
   - Ensure the client ID is correct

2. **"Email domain not allowed"**
   - Check the `ALLOWED_DOMAINS` setting
   - Ensure the user's email domain is in the allowed list

3. **"Invalid NextAuth token"**
   - Verify `NEXTAUTH_SECRET` is set correctly
   - Check that the frontend and backend use the same secret

### Debug Mode

Enable verbose logging to troubleshoot authentication issues:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## Integration Examples

### React with NextAuth.js

```jsx
import { useSession } from 'next-auth/react';

function ApiComponent() {
  const { data: session } = useSession();
  
  const callAPI = async () => {
    const response = await fetch('/api/backend/v1/files', {
      headers: {
        'Authorization': `Bearer ${session.accessToken}`
      }
    });
    return response.json();
  };
  
  return (
    <div>
      {session ? (
        <button onClick={callAPI}>Call API</button>
      ) : (
        <p>Please sign in</p>
      )}
    </div>
  );
}
```

### Vanilla JavaScript

```javascript
// After Google OAuth login
function callAPIWithToken(idToken) {
  fetch('/v1/auth/validate-token', {
    method: 'GET',
    headers: {
      'Authorization': `Bearer ${idToken}`,
      'Content-Type': 'application/json'
    }
  })
  .then(response => response.json())
  .then(data => {
    if (data.valid) {
      console.log('User:', data.user);
      // Proceed with authenticated requests
    } else {
      console.error('Token validation failed');
    }
  });
}
```

## Need Help?

1. Check the server logs for detailed error messages
2. Use the `/v1/auth/auth-config` endpoint to verify your configuration
3. Test with the development mode first before switching to production
4. Visit `http://localhost:8000/home` for quick access to authentication endpoints 