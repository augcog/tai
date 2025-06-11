# Example Requests for Postman

This directory contains example JSON request bodies that you can use directly with Postman to test the API endpoints.

## Completions API

### Non-RAG Request

`non_rag_request.json` - A non-streaming completion request without RAG enabled.

```json
{
  "course": "default",
  "messages": [
    {
      "role": "system",
      "content": "You are a helpful assistant that provides information about the UC Berkeley TAI project."
    },
    {
      "role": "user",
      "content": "Tell me about the UC Berkeley TAI project."
    }
  ],
  "temperature": 0.7,
  "stream": false,
  "rag": false
}
```

### RAG Request

`rag_request.json` - A non-streaming completion request with RAG enabled.

```json
{
  "course": "default",
  "messages": [
    {
      "role": "system",
      "content": "You are a helpful assistant that provides information about the UC Berkeley TAI project."
    },
    {
      "role": "user",
      "content": "Tell me about the UC Berkeley TAI project."
    }
  ],
  "temperature": 0.7,
  "stream": false,
  "rag": true
}
```

### Streaming Request

`streaming_request.json` - A streaming completion request with RAG enabled.

```json
{
  "course": "default",
  "messages": [
    {
      "role": "user",
      "content": "What is the UC Berkeley TAI project about?"
    }
  ],
  "temperature": 0.7,
  "stream": true,
  "rag": true
}
```

### CS61A Request

`cs61a_request.json` - A course-specific request for CS61A.

```json
{
  "course": "CS61A",
  "messages": [
    {
      "role": "system",
      "content": "You are a helpful assistant for the CS61A course at UC Berkeley."
    },
    {
      "role": "user",
      "content": "What topics are covered in CS61A?"
    }
  ],
  "temperature": 0.7,
  "stream": false,
  "rag": true
}
```

## How to Use

1. Open Postman
2. Create a new request to your API endpoint (e.g., `http://localhost:8000/v1/completions`)
3. Set the request method to `POST`
4. Set the Content-Type header to `application/json`
5. In the request body, select "raw" and "JSON" format
6. Copy and paste the content of one of these JSON files
7. Send the request

## Important Notes

- Make sure your server is running before sending requests
- For streaming requests, Postman may not display the streamed response correctly
- The `course` field is required for all requests
- The `messages` field must contain at least one message
- The `temperature` and `stream` fields are required
- The `rag` field is optional and defaults to `true`
