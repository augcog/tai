# API Authentication Guide

This document provides comprehensive guidance on authenticating with the Course AI Assistant API for frontend developers.

## Authentication Overview

The Course AI Assistant API uses token-based authentication to secure all endpoints. The authentication system works differently depending on the environment:

- **Development Environment**: Simplified authentication with mock tokens
- **Production Environment**: Google OAuth 2.0 authentication with real tokens

All API endpoints require authentication via an `Authorization` header or, for certain file-related endpoints, an `auth_token` query parameter.

## Development Mode Authentication

In development mode, authentication is simplified to facilitate easier testing and integration.

### Configuration

Development mode is enabled when the `auth_required` setting is set to `false` in the backend configuration. This is typically done in the `.env` file:

```
auth_required=false
```

### Using Mock Tokens

When in development mode, you can use any token with the format `Bearer <token>`. The recommended approach is to use:

```
Bearer mock_test_token
```

### Example Request (Development)

```javascript
// JavaScript fetch example for development
async function fetchFiles() {
  const response = await fetch('/v1/local-files', {
    method: 'GET',
    headers: {
      'Authorization': 'Bearer mock_test_token'
    }
  });
  
  const data = await response.json();
  return data;
}
```

## Production Authentication

In production, the API uses Google OAuth 2.0 for authentication. This provides secure, industry-standard authentication.

### Google OAuth 2.0 Integration

To authenticate with the API in production:

1. **Set Up Google OAuth in Your Frontend**:
   - Register your application with Google Cloud Console
   - Configure the OAuth consent screen
   - Create OAuth 2.0 client credentials

2. **Implement Google Sign-In**:
   - Use Google's authentication libraries to implement sign-in
   - Obtain an ID token after successful authentication

3. **Send the ID Token with API Requests**:
   - Include the ID token in the `Authorization` header
   - Format: `Bearer <google_id_token>`

### Example Implementation (Production)

```javascript
// Example using Google's OAuth client library
async function authenticateAndFetchFiles() {
  // 1. Initialize Google Auth
  const auth2 = await gapi.auth2.init({
    client_id: 'YOUR_GOOGLE_CLIENT_ID',
    scope: 'profile email'
  });
  
  // 2. Sign in and get ID token
  const googleUser = await auth2.signIn();
  const idToken = googleUser.getAuthResponse().id_token;
  
  // 3. Use the ID token for API requests
  const response = await fetch('/v1/local-files', {
    method: 'GET',
    headers: {
      'Authorization': `Bearer ${idToken}`
    }
  });
  
  const data = await response.json();
  return data;
}
```

## File Endpoint Authentication

For file endpoints (e.g., retrieving files or images), you can authenticate using either:

1. **Authorization Header** (preferred for programmatic access)
2. **Query Parameter** (useful for embedded resources like images)

### Query Parameter Authentication

When embedding resources in HTML (like images or PDFs), you can use the `auth_token` query parameter:

```html
<img src="/v1/local-files/images/example.jpg?auth_token=YOUR_TOKEN" alt="Example Image">
```

This is particularly useful when the browser makes separate requests for resources and you cannot set headers.

## Authentication Flow Diagram

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│             │     │             │     │             │
│  Frontend   │────▶│  Google     │────▶│  Backend    │
│  Application│     │  OAuth      │     │  API        │
│             │◀────│             │◀────│             │
└─────────────┘     └─────────────┘     └─────────────┘
       │                                       ▲
       │                                       │
       │                                       │
       └───────────────────────────────────────┘
                Direct API calls with token
```

## Common Authentication Errors

| Status Code | Error Message | Possible Cause | Solution |
|-------------|---------------|----------------|----------|
| 401 | "Invalid Google token" | Expired or invalid token | Refresh the token or re-authenticate |
| 401 | "Authorization header missing" | No Authorization header | Add the Authorization header |
| 403 | "Access denied" | Valid token but insufficient permissions | Check user permissions |

## Best Practices

1. **Token Storage**:
   - Store tokens securely (e.g., in memory or secure HTTP-only cookies)
   - Never store tokens in localStorage or sessionStorage for production

2. **Token Refresh**:
   - Implement token refresh logic to handle token expiration
   - Google ID tokens typically expire after 1 hour

3. **Error Handling**:
   - Gracefully handle authentication errors
   - Redirect to login page when authentication fails

4. **Security Headers**:
   - Use HTTPS for all API requests
   - Implement proper CORS policies

## Testing Authentication

For testing your frontend integration:

1. **Development Environment**:
   - Use `Bearer mock_test_token` for all requests
   - Verify that responses include the expected data

2. **Production Testing**:
   - Use a test Google account
   - Verify token acquisition and API access

## Transitioning from Development to Production

When moving from development to production:

1. Replace the mock token with real Google authentication
2. Update environment variables on the backend (`auth_required=true`)
3. Configure the correct Google client ID in both frontend and backend
4. Test the complete authentication flow

## Need Help?

If you encounter authentication issues:

1. Check the API logs for detailed error messages
2. Verify your token format and validity
3. Ensure your Google OAuth configuration is correct
4. Contact the backend team for assistance
