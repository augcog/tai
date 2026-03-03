# Batch File Upload and Conversion API

This module provides a FastAPI-based web service for batch uploading and converting files with real-time progress tracking via Server-Sent Events (SSE).

## Quick Start

### Starting the Server

```bash
cd rag

# Option 1: Run directly with Python
python -m file_conversion_router.web.app

# Option 2: Run with uvicorn (recommended for development)
uvicorn file_conversion_router.web.app:app --host 0.0.0.0 --port 8001 --reload

# Option 3: Run with custom settings
uvicorn file_conversion_router.web.app:app --host 0.0.0.0 --port 8001 --workers 4
```

The API will be available at `http://localhost:8001`. API documentation is available at `http://localhost:8001/docs`.

---

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/batch/upload` | Upload files and start batch conversion |
| GET | `/batch/{job_id}/stream` | Stream real-time progress via SSE |
| GET | `/batch/{job_id}/status` | Get current job status (polling) |
| POST | `/batch/{job_id}/cancel` | Cancel a running job |
| GET | `/batch/jobs` | List recent jobs |
| DELETE | `/batch/{job_id}` | Delete a completed job |
| GET | `/health` | Health check |
| GET | `/storage/stats` | Get temp storage statistics |

---

## Usage Examples

### 1. Upload Files (cURL)

```bash
# Upload single file
curl -X POST "http://localhost:8001/batch/upload" \
  -F "files=@document.pdf" \
  -F "course_code=CS61A" \
  -F "course_name=Structure and Interpretation of Computer Programs"

# Upload multiple files
curl -X POST "http://localhost:8001/batch/upload" \
  -F "files=@lecture01.pdf" \
  -F "files=@lecture02.pdf" \
  -F "files=@notes.md" \
  -F "course_code=CS61A" \
  -F "course_name=Structure and Interpretation of Computer Programs" \
  -F "auto_embed=true"

# Upload with custom output directory
curl -X POST "http://localhost:8001/batch/upload" \
  -F "files=@document.pdf" \
  -F "course_code=CS61A" \
  -F "course_name=Structure and Interpretation of Computer Programs" \
  -F "output_dir=/path/to/custom/output" \
  -F "db_path=/path/to/custom/database.db"
```

**Response:**
```json
{
  "job_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "pending",
  "files_received": 3,
  "files_info": [
    {"file_name": "lecture01.pdf", "file_size": 1048576, "content_type": "application/pdf"},
    {"file_name": "lecture02.pdf", "file_size": 2097152, "content_type": "application/pdf"},
    {"file_name": "notes.md", "file_size": 4096, "content_type": "text/markdown"}
  ],
  "message": "Batch job created. 3 files queued for processing."
}
```

### 2. Stream Progress (SSE)

```bash
# Connect to SSE stream
curl -N "http://localhost:8001/batch/550e8400-e29b-41d4-a716-446655440000/stream"
```

**SSE Events:**
```
event: job_start
data: {"event_type": "job_start", "job_id": "550e8400...", "total_files": 3}

event: file_start
data: {"event_type": "file_start", "job_id": "550e8400...", "file_name": "lecture01.pdf"}

event: file_done
data: {"event_type": "file_done", "job_id": "550e8400...", "file_name": "lecture01.pdf", "file_status": "completed"}

event: file_start
data: {"event_type": "file_start", "job_id": "550e8400...", "file_name": "lecture02.pdf"}

event: file_error
data: {"event_type": "file_error", "job_id": "550e8400...", "file_name": "lecture02.pdf", "error_message": "PDF extraction failed"}

event: job_complete
data: {"event_type": "job_complete", "job_id": "550e8400...", "result": {...}}
```

### 3. Check Job Status (Polling)

```bash
curl "http://localhost:8001/batch/550e8400-e29b-41d4-a716-446655440000/status"
```

**Response:**
```json
{
  "job_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "processing",
  "total_files": 3,
  "processed_files": 1,
  "successful_files": 1,
  "failed_files": 0,
  "skipped_files": 0,
  "current_file": "lecture02.pdf",
  "errors": [],
  "results": [
    {"file_name": "lecture01.pdf", "file_uuid": "abc123...", "chunks_count": 15, "status": "completed"}
  ],
  "created_at": "2024-01-15T10:30:00",
  "updated_at": "2024-01-15T10:30:45"
}
```

### 4. Cancel a Job

```bash
curl -X POST "http://localhost:8001/batch/550e8400-e29b-41d4-a716-446655440000/cancel"
```

### 5. List Recent Jobs

```bash
curl "http://localhost:8001/batch/jobs?limit=10"
```

---

## Python Usage

### Basic Usage

```python
import requests

# Upload files
files = [
    ('files', ('lecture01.pdf', open('lecture01.pdf', 'rb'), 'application/pdf')),
    ('files', ('notes.md', open('notes.md', 'rb'), 'text/markdown')),
]
data = {
    'course_code': 'CS61A',
    'course_name': 'Structure and Interpretation of Computer Programs',
    'auto_embed': 'true',
}

response = requests.post('http://localhost:8001/batch/upload', files=files, data=data)
result = response.json()
job_id = result['job_id']
print(f"Job created: {job_id}")
```

### Stream Progress with SSE

```python
import requests
import json

def stream_progress(job_id: str):
    """Stream progress updates from the batch job."""
    url = f'http://localhost:8001/batch/{job_id}/stream'

    with requests.get(url, stream=True) as response:
        for line in response.iter_lines():
            if line:
                line = line.decode('utf-8')
                if line.startswith('data:'):
                    data = json.loads(line[5:].strip())
                    event_type = data.get('event_type')

                    if event_type == 'job_start':
                        print(f"Starting processing {data['total_files']} files...")
                    elif event_type == 'file_start':
                        print(f"Processing: {data['file_name']}")
                    elif event_type == 'file_done':
                        print(f"Completed: {data['file_name']} ({data['file_status']})")
                    elif event_type == 'file_error':
                        print(f"Failed: {data['file_name']} - {data['error_message']}")
                    elif event_type == 'job_complete':
                        result = data.get('result', {})
                        print(f"\nJob complete!")
                        print(f"  Processed: {result.get('files_processed', 0)}")
                        print(f"  Failed: {result.get('files_failed', 0)}")
                        print(f"  Skipped: {result.get('files_skipped', 0)}")
                        break

# Usage
stream_progress(job_id)
```

### Async Usage with httpx

```python
import httpx
import asyncio
import json

async def upload_and_track(files_paths: list, course_code: str, course_name: str):
    """Upload files and track progress asynchronously."""
    async with httpx.AsyncClient() as client:
        # Upload files
        files = [('files', open(f, 'rb')) for f in files_paths]
        data = {
            'course_code': course_code,
            'course_name': course_name,
        }

        response = await client.post(
            'http://localhost:8001/batch/upload',
            files=files,
            data=data,
        )
        result = response.json()
        job_id = result['job_id']
        print(f"Job created: {job_id}")

        # Stream progress
        async with client.stream('GET', f'http://localhost:8001/batch/{job_id}/stream') as response:
            async for line in response.aiter_lines():
                if line.startswith('data:'):
                    data = json.loads(line[5:].strip())
                    print(f"{data['event_type']}: {data.get('file_name', '')}")
                    if data['event_type'] == 'job_complete':
                        return data.get('result')

# Run
result = asyncio.run(upload_and_track(
    ['doc1.pdf', 'doc2.pdf'],
    'CS61A',
    'Structure and Interpretation of Computer Programs'
))
```

---

## Programmatic API (Without Web Server)

You can also use the batch conversion function directly without the web server:

```python
from pathlib import Path
from file_conversion_router.api import batch_convert_files

def on_progress(file_name: str, status: str, error: str = None):
    """Progress callback."""
    if status == 'started':
        print(f"Processing: {file_name}")
    elif status == 'completed':
        print(f"Completed: {file_name}")
    elif status == 'failed':
        print(f"Failed: {file_name} - {error}")
    elif status == 'skipped':
        print(f"Skipped (cached): {file_name}")

# Convert files
result = batch_convert_files(
    files=[
        Path('/path/to/lecture01.pdf'),
        Path('/path/to/lecture02.pdf'),
        Path('/path/to/notes.md'),
    ],
    course_code='CS61A',
    course_name='Structure and Interpretation of Computer Programs',
    output_dir=Path('/path/to/output'),
    db_path=Path('/path/to/database.db'),
    auto_embed=True,
    progress_callback=on_progress,
)

print(f"\nResults:")
print(f"  Processed: {result['files_processed']}")
print(f"  Failed: {result['files_failed']}")
print(f"  Skipped: {result['files_skipped']}")
print(f"  Total chunks: {result['total_chunks']}")
```

---

## Configuration

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `RAG_TEMP_DIR` | Temporary upload directory | `/tmp/rag_batch_uploads` |

### Upload Limits

Located in `config.py`:

```python
BATCH_UPLOAD_CONFIG = {
    "max_file_size_mb": 100,        # Max size per file
    "max_total_size_mb": 500,       # Max total upload size
    "max_files_per_batch": 100,     # Max files per batch
    "allowed_extensions": [
        ".pdf", ".md", ".html", ".ipynb", ".py", ".rst",
        ".mp4", ".mkv", ".webm", ".mov"
    ],
    "temp_dir": Path("/tmp/rag_batch_uploads"),
    "cleanup_delay_seconds": 3600,  # Cleanup temp files after 1 hour
}
```

---

## Supported File Types

| Extension | Type | Converter |
|-----------|------|-----------|
| `.pdf` | PDF documents | PdfConverter |
| `.md` | Markdown | MarkdownConverter |
| `.html` | HTML pages | HtmlConverter |
| `.ipynb` | Jupyter notebooks | NotebookConverter |
| `.py` | Python files | PythonConverter |
| `.rst` | reStructuredText | RstConverter |
| `.mp4`, `.mkv`, `.webm`, `.mov` | Video files | VideoConverter |

---

## Job Status Values

| Status | Description |
|--------|-------------|
| `pending` | Job created, waiting to start |
| `processing` | Currently processing files |
| `completed` | All files processed successfully |
| `failed` | Job failed with critical error |
| `cancelled` | Job was cancelled by user |

## File Status Values

| Status | Description |
|--------|-------------|
| `pending` | File waiting to be processed |
| `processing` | File currently being processed |
| `completed` | File converted successfully |
| `failed` | File conversion failed |
| `skipped` | File skipped (already in cache) |

---

## Error Handling

### Validation Errors (HTTP 400)

Returned when files fail validation before processing:

```json
{
  "detail": {
    "message": "No valid files in batch",
    "errors": [
      {"file_name": "doc.txt", "error_type": "invalid_extension", "message": "Unsupported file type: .txt"},
      {"file_name": "huge.pdf", "error_type": "file_too_large", "message": "File too large: 150MB exceeds 100MB limit"}
    ]
  }
}
```

### Processing Errors

Files that fail during conversion are recorded in the job results:

```json
{
  "errors": [
    {"file_name": "corrupted.pdf", "error_message": "PDF extraction failed: Invalid PDF structure"}
  ],
  "results": [
    {"file_name": "corrupted.pdf", "status": "failed", "error": "PDF extraction failed..."}
  ]
}
```

The batch continues processing other files even when some fail.

---

## Best Practices

1. **Use SSE for real-time updates**: Connect to `/batch/{job_id}/stream` for live progress tracking.

2. **Implement reconnection logic**: If SSE connection drops, reconnect or fall back to polling `/batch/{job_id}/status`.

3. **Handle large batches**: For many files, consider splitting into smaller batches of 20-50 files.

4. **Monitor temp storage**: Use `/storage/stats` to monitor temporary storage usage.

5. **Clean up completed jobs**: Use `DELETE /batch/{job_id}` to free up resources after processing.

6. **Set appropriate timeouts**: Large files (especially videos) may take longer to process.

---

## Troubleshooting

### Job stuck in "processing"

- Check server logs for errors
- The file might be very large or complex
- Try cancelling and resubmitting

### SSE connection drops

- Implement automatic reconnection in your client
- Use the `/status` endpoint as fallback
- Check for proxy/load balancer timeout settings

### Files not being processed

- Verify file extension is in allowed list
- Check file size limits
- Ensure file is not corrupted

### High memory usage

- Process files in smaller batches
- Increase server memory
- Check for memory leaks in converters
