# Files API Documentation

## Quick Start

### 1. Add Files
```bash
# Drop files into the data directory
cp my_file.pdf data/CS61A/documents/
cp lecture.mp4 data/CS61A/videos/
```

### 2. Use API
```bash
# List all files (auto-discovery happens automatically)
curl "http://localhost:8000/v1/files"

# Filter by course
curl "http://localhost:8000/v1/files?course_code=CS61A"

# Search files
curl "http://localhost:8000/v1/files?search=lab"

# Download file securely
curl "http://localhost:8000/v1/files/{uuid}/download"
```

## API Endpoints

### List Files
```
GET /v1/files
```

**Parameters:**
- `course_code` (optional) - Filter by course (e.g., CS61A)
- `category` (optional) - Filter by category (document, video, audio, other)
- `search` (optional) - Search in filename and title
- `page` (optional) - Page number (default: 1)
- `limit` (optional) - Items per page (default: 100)

**Response:**
```json
{
  "files": [
    {
      "uuid": "550e8400-e29b-41d4-a716-446655440000",
      "filename": "lab_01.pdf",
      "title": "Lab 01",
      "course": "CS61A",
      "category": "document",
      "size_bytes": 1048576,
      "mime_type": "application/pdf",
      "created_at": "2023-01-01T00:00:00Z",
      "modified_at": "2023-01-01T00:00:00Z"
    }
  ],
  "total_count": 1,
  "page": 1,
  "limit": 100,
  "has_next": false,
  "has_prev": false,
  "filters_applied": {
    "course_code": "CS61A",
    "category": null,
    "search": null
  }
}
```

### Get File Metadata
```
GET /v1/files/{uuid}
```

Returns detailed metadata for a specific file.

### Download File
```
GET /v1/files/{uuid}/download
```

Downloads the file securely. Returns the file with proper headers.

### Get Statistics
```
GET /v1/files/stats/summary
```

Returns file system statistics including total files, courses, and system info.

## Features

### Auto-Discovery
- Files are automatically discovered when added to the data directory
- No manual rescans needed
- New files appear in API responses immediately

### Security
- UUID-based file access (no path exposure)
- Path traversal protection
- Secure file serving

### Simple Metadata
- **Course** - Extracted from directory structure (CS61A, CS61B, etc.)
- **Category** - Basic file type (document, video, audio, other)
- **Title** - Clean title generated from filename

### Directory Structure
```
data/
├── CS61A/
│   ├── documents/     # → category: document
│   ├── videos/        # → category: video
│   ├── audios/        # → category: audio
│   └── others/        # → category: other
└── CS61B/
    ├── documents/
    └── videos/
```

## Examples

### Filter by Course
```bash
curl "http://localhost:8000/v1/files?course_code=CS61A"
```

### Filter by Category
```bash
curl "http://localhost:8000/v1/files?category=document"
```

### Search Files
```bash
curl "http://localhost:8000/v1/files?search=recursion"
```

### Combined Filters
```bash
curl "http://localhost:8000/v1/files?course_code=CS61A&category=document&search=lab"
```

### Pagination
```bash
curl "http://localhost:8000/v1/files?page=2&limit=50"
```

## Error Handling

### File Not Found
```json
{
  "detail": "File not found"
}
```

### Invalid Parameters
```json
{
  "detail": "Invalid page number"
}
```

## Testing

Run the test suite:
```bash
python3.10 -m pytest tests/unit_tests/test_api/test_v1/test_endpoints/test_files.py -v
```

All tests should pass (17/17).

## Architecture

### Clean Design
- **Single API** - All file operations in `/v1/files`
- **No duplications** - Course management is in `/v1/courses`
- **Simple filtering** - Only essential parameters
- **UUID security** - Secure file access without path exposure

### File Service
- Auto-discovery of new files
- Simple metadata extraction
- Efficient caching
- Database integration

### Schemas
- Clean, simple response formats
- Essential metadata only
- Good examples for frontend integration

## Integration

### Postman Collection
📋 **Complete Postman collection available at `postman/files_api_collection.json`**

- **Comprehensive examples** with realistic data
- **All endpoints covered** with diverse scenarios
- **Frontend-friendly** examples for easy integration
- **Real file examples** using actual data directory files

Import the collection and set environment variables:
- `baseUrl`: `http://localhost:8000`
- `authToken`: Your auth token (if needed)

### Frontend Usage
```javascript
// List files
const response = await fetch('/v1/files?course_code=CS61A');
const data = await response.json();

// Download file
const fileUrl = `/v1/files/${fileUuid}/download`;
window.open(fileUrl);
```

### Authentication
Files API uses the same authentication as other endpoints. In development mode, authentication may be disabled.

## Maintenance

### Adding New Files
1. Drop files into appropriate directory structure
2. Files are auto-discovered on next API call
3. No manual intervention needed

### Monitoring
- Check `/v1/files/stats/summary` for system health
- Monitor file counts and discovery status
- All operations are logged

## Support

For issues or questions:
1. Check test results to ensure API is working
2. Verify file permissions and directory structure
3. Check logs for auto-discovery status
