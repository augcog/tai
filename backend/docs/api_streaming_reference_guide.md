# Chat Completions API with References

## Overview

The Chat Completions API provides OpenAI-compatible endpoints for generating assistant responses with citations and references. This API is designed for seamless integration with frontend applications, supporting both streaming and non-streaming responses with reference handling.

> **Important**: For authentication requirements and implementation details, please refer to the [Authentication Guide](authentication.md).
>
> For information about accessing and displaying files, please refer to the [Local File API Guide](local_file_api.md).

## Features

- OpenAI-compatible API format for easy integration with existing clients
- Streaming and non-streaming response modes
- Retrieval-augmented generation with inline references
- Citation and reference handling via tool calls
- Course-specific content adaptation

## API Endpoints

### Chat Completions

```
POST /v1/completions
```

Generates chat completions with optional references and citations.

**Request Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `course` | string | Course code (e.g., "CS61A") |
| `messages` | array | Array of message objects with `role` and `content` |
| `temperature` | number | (Optional) Controls randomness, 0-1 |
| `stream` | boolean | (Optional) Enable streaming responses |
| `rag` | boolean | (Optional) Enable retrieval-augmented generation with references |

**Example Request:**

```json
{
  "course": "CS61A",
  "messages": [
    {"role": "user", "content": "What is the UC Berkeley TAI project?"}
  ],
  "temperature": 0.7,
  "stream": true,
  "rag": true
}
```

## Response Format

### Non-Streaming Response

When `stream: false`, the API returns a complete response with content and any reference tool calls:

```json
{
  "id": "chatcmpl-123abc",
  "object": "chat.completion.chunk",
  "created": 1694268190,
  "model": "custom-model",
  "system_fingerprint": "fp_custom",
  "choices": [{
    "index": 0,
    "delta": {
      "role": "assistant",
      "content": "UC Berkeley TAI (Teaching Assistant Intelligence) is a project that aims to [1] develop AI teaching assistants.",
      "tool_calls": [
        {
          "id": "call_abc123",
          "type": "function",
          "function": {
            "name": "add_reference",
            "arguments": "{\"number\":1,\"title\":\"UC Berkeley TAI\",\"url\":\"https://github.com/augcog/tai\"}"
          }
        }
      ]
    },
    "finish_reason": "stop"
  }]
}
```

### Streaming Response

When `stream: true`, the API returns a series of chunks following OpenAI's streaming format:

1. Initial role chunk
2. Multiple content chunks
3. Tool call chunks (for references)
4. Final completion chunk

See the [Implementation Guide](#implementation-guide) section for detailed examples of each chunk.

## Reference System

### How References Work

The API includes references in the response using OpenAI-compatible function calls:

1. Numbers in square brackets (e.g., `[1]`, `[2]`) appear in the content text
2. Each reference number corresponds to a tool call with citation details
3. Frontend applications should render these as clickable links

### Reference Tool Call Format

```json
{
  "id": "call_abc123",
  "type": "function",
  "function": {
    "name": "add_reference",
    "arguments": "{\"number\":1,\"title\":\"UC Berkeley TAI\",\"url\":\"https://github.com/augcog/tai\"}"
  }
}
```

The `arguments` field contains:
- `number`: Reference number matching the marker in the text
- `title`: Reference title
- `url`: URL to the reference source

## Implementation Guide

### Implementation Flow

```
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│  Frontend   │──────▶  Chat API   │──────▶  Response   │
│ Application │      │   Request   │      │ Processing  │
└─────────────┘      └─────────────┘      └─────────────┘
       │                                         │
       │                                         ▼
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│  Reference  │◀─────┤  Content    │◀─────┤  Streaming/ │
│  Rendering  │      │  Display    │      │ Non-Stream  │
└─────────────┘      └─────────────┘      └─────────────┘
```

### Non-Streaming Implementation

```javascript
async function fetchCompletion() {
  const response = await fetch('/v1/completions', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Bearer YOUR_AUTH_TOKEN'
    },
    body: JSON.stringify({
      course: "CS61A",
      messages: [{ role: "user", content: "What is the UC Berkeley TAI project?" }],
      stream: false,
      rag: true
    })
  });

  const data = await response.json();
  const content = data.choices[0].delta.content;

  // Process reference tool calls
  const references = [];
  if (data.choices[0].delta.tool_calls) {
    for (const toolCall of data.choices[0].delta.tool_calls) {
      if (toolCall.function.name === 'add_reference') {
        const args = JSON.parse(toolCall.function.arguments);
        references.push({
          number: args.number,
          title: args.title,
          url: args.url
        });
      }
    }
  }

  // Render content with reference links
  displayContent(content, references);
}

function displayContent(content, references) {
  // Create a map of reference numbers to references
  const referenceMap = {};
  references.forEach(ref => {
    referenceMap[ref.number] = ref;
  });

  // Replace reference markers with links
  const contentWithLinks = content.replace(/\[(\d+)\]/g, (match, number) => {
    const ref = referenceMap[parseInt(number)];
    if (ref) {
      return `<a href="${ref.url}" class="reference" data-reference="${number}" title="${ref.title}">[${number}]</a>`;
    }
    return match;
  });

  // Display in the UI
  document.getElementById('response').innerHTML = contentWithLinks;
  
  // Add reference list at the bottom
  if (references.length > 0) {
    const referenceList = document.createElement('div');
    referenceList.className = 'reference-list';
    
    const heading = document.createElement('h3');
    heading.textContent = 'References';
    referenceList.appendChild(heading);
    
    const list = document.createElement('ol');
    references.forEach(ref => {
      const item = document.createElement('li');
      item.id = `ref-${ref.number}`;
      item.innerHTML = `<a href="${ref.url}" target="_blank">${ref.title}</a>`;
      list.appendChild(item);
    });
    
    referenceList.appendChild(list);
    document.getElementById('references-container').appendChild(referenceList);
  }
}
```

### Streaming Implementation

```javascript
async function streamCompletion() {
  const response = await fetch('/v1/completions', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Bearer YOUR_AUTH_TOKEN'
    },
    body: JSON.stringify({
      course: "CS61A",
      messages: [{ role: "user", content: "What is the UC Berkeley TAI project?" }],
      stream: true,
      rag: true
    })
  });

  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let content = '';
  const references = [];

  // Display area setup
  const responseElement = document.getElementById('response');
  responseElement.innerHTML = '';

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    
    // Decode the chunk and split by double newlines
    const chunk = decoder.decode(value);
    const lines = chunk.split('\n\n');
    
    for (const line of lines) {
      if (line.startsWith('data: ')) {
        try {
          const data = JSON.parse(line.substring(6));
          
          // Handle role initialization
          if (data.choices[0].delta.role) {
            // Role initialization, nothing to display yet
          }
          
          // Handle content updates
          if (data.choices[0].delta.content) {
            content += data.choices[0].delta.content;
            responseElement.textContent = content;
          }
          
          // Handle tool calls (references)
          if (data.choices[0].delta.tool_calls) {
            for (const toolCall of data.choices[0].delta.tool_calls) {
              if (toolCall.function.name === 'add_reference') {
                const args = JSON.parse(toolCall.function.arguments);
                references.push({
                  number: args.number,
                  title: args.title,
                  url: args.url
                });
              }
            }
          }
          
          // Handle completion
          if (data.choices[0].finish_reason === 'stop') {
            // Apply reference formatting after stream completes
            finalizeContent(content, references);
          }
        } catch (error) {
          console.error('Error parsing chunk:', error);
        }
      }
    }
  }
}

function finalizeContent(content, references) {
  // Create a map of reference numbers to references
  const referenceMap = {};
  references.forEach(ref => {
    referenceMap[ref.number] = ref;
  });

  // Replace reference markers with links
  const contentWithLinks = content.replace(/\[(\d+)\]/g, (match, number) => {
    const ref = referenceMap[parseInt(number)];
    if (ref) {
      return `<a href="${ref.url}" class="reference" data-reference="${number}" title="${ref.title}">[${number}]</a>`;
    }
    return match;
  });

  // Display formatted content
  document.getElementById('response').innerHTML = contentWithLinks;
  
  // Render references list
  renderReferencesList(references);
}

function renderReferencesList(references) {
  if (references.length === 0) return;
  
  const container = document.getElementById('references-container');
  container.innerHTML = '<h3>References</h3><ol></ol>';
  
  const list = container.querySelector('ol');
  references.forEach(ref => {
    const item = document.createElement('li');
    item.id = `ref-${ref.number}`;
    item.innerHTML = `<a href="${ref.url}" target="_blank">${ref.title}</a>`;
    list.appendChild(item);
  });
}
```

## CSS Styling for References

Here's a recommended CSS styling for references in your frontend application:

```css
/* Reference link styling */
.reference {
  color: #0066cc;
  text-decoration: none;
  font-weight: 500;
  cursor: pointer;
  background-color: rgba(0, 102, 204, 0.1);
  padding: 0.1em 0.2em;
  border-radius: 3px;
}

.reference:hover {
  text-decoration: underline;
  background-color: rgba(0, 102, 204, 0.2);
}

/* References list styling */
.reference-list {
  margin-top: 2rem;
  padding-top: 1rem;
  border-top: 1px solid #eaeaea;
}

.reference-list h3 {
  margin-bottom: 0.5rem;
  font-size: 1.2rem;
}

.reference-list ol {
  padding-left: 1.5rem;
}

.reference-list li {
  margin-bottom: 0.5rem;
}

.reference-list a {
  color: #0066cc;
  text-decoration: none;
}

.reference-list a:hover {
  text-decoration: underline;
}
```

## Best Practices

1. **Error Handling**:
   - Implement proper error handling for both streaming and non-streaming requests
   - Display user-friendly error messages when API calls fail

2. **Reference Rendering**:
   - Always render references as clickable links
   - Provide a references section below the content for better accessibility
   - Consider implementing hover tooltips for reference previews

3. **Streaming Performance**:
   - Use efficient DOM updates to prevent layout thrashing during streaming
   - Consider throttling UI updates for very fast streams

4. **Accessibility**:
   - Ensure reference links have proper ARIA attributes
   - Make reference numbers distinct and easily clickable

## Common Issues and Solutions

| Issue | Solution |
|-------|----------|
| References not appearing | Check that `rag` parameter is set to `true` |
| Streaming freezes | Implement proper error handling and reconnection logic |
| Reference links not working | Verify the reference URL format and implement proper error fallbacks |
| Slow performance with many references | Implement virtualized rendering for large reference lists |

## Need Help?

If you encounter issues implementing the Chat Completions API with references:

1. Check the API request format and parameters
2. Verify that streaming is properly set up if using that mode
3. Inspect network requests to ensure the API is returning the expected format
4. Refer to the complete code examples in the [examples directory](./examples/)
5. Contact the backend team for assistance