# Course AI Assistant API Documentation

Welcome to the Course AI Assistant API documentation. This directory contains comprehensive guides for integrating with and using the API.

## Available Documentation

### Core Documentation

- [**Frontend Integration Guide**](frontend_integration.md) - Detailed guide on integrating with the API, focusing on handling chat responses and references
- [**Authentication Guide**](authentication.md) - Comprehensive documentation on authenticating with the API in both development and production environments
- [**Local File API Guide**](local_file_api.md) - Complete documentation for the Local File API endpoints and features

### API Endpoints

The Course AI Assistant API provides several key endpoints:

1. **Chat Completions API**
   - `POST /v1/completions` - Generate AI responses to user messages

2. **Local File API**
   - `GET /v1/local-files` - List files in a directory
   - `GET /v1/local-files/hierarchy` - Get a hierarchical tree structure of files and directories
   - `GET /v1/local-files/{file_path}` - Retrieve a specific file
   - `GET /v1/local-files/categories` - List available file categories
   - `GET /v1/local-files/folders` - List available file folders

## Getting Started

If you're new to the API, we recommend starting with:

1. First, read the [Authentication Guide](authentication.md) to understand how to authenticate with the API
2. Then, follow the [Frontend Integration Guide](frontend_integration.md) to learn how to make requests and handle responses

## API Testing

You can test the API using:

- The Swagger UI documentation at `/docs`
- The ReDoc documentation at `/redoc`
- The File API Tester at `/file-tester`

## Need Help?

If you encounter any issues or have questions about the API:

1. Check the documentation in this directory
2. Look for error messages in the API responses
3. Contact the backend team for assistance
