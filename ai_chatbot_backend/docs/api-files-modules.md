# Files & Modules API Reference

Base URL: `/api`

All endpoints require a valid API token in the `Authorization` header (Bearer token).

---

## Overview

TAI organizes course content into **files** and **modules**:

- **Files** are individual documents (PDFs, HTML pages, Python files, videos) stored in the `metadata.db` database. Every file has a UUID, a `relative_path` showing its location in the course directory tree, and structured `sections` extracted from the content.
- **Modules** are logical groupings of files derived from the course directory structure. Each module has a stable UUID that won't change across server restarts. Modules follow a fixed hierarchy:

```
{course_dir}/
  practice/
    hw/hw01/        -> module (category: practice)
    hw/hw02/        -> module
    lab/lab00/       -> module
    proj/hog/        -> module
  study/
    lec01/           -> module (category: study)
    lec02/           -> module
  support/
    articles/        -> module (category: support)
    resources/       -> module
```

**When should the frontend use modules vs files?**
- Use **modules** to build a sidebar/navigation tree showing course structure (practice > hw > hw01, study > lec01, etc.)
- Use **files** when you need the actual content — file details, download links, or to pass a `file_uuid` to the chat API
- Use `module_uuid` in the **chat completions API** to scope RAG retrieval to a specific module (e.g., only search within lab01 materials)

---

## Modules API

### List Modules

```
GET /api/modules?course_code={course_code}
```

Returns all modules for a course, sorted by path.

**Query Parameters:**

| Parameter     | Type   | Required | Description                      |
|---------------|--------|----------|----------------------------------|
| `course_code` | string | yes      | Course code, e.g. `CS 61A`      |

**Response:**

```json
{
  "modules": [
    {
      "module_uuid": "7c49c2d9-89b9-43a5-9732-9b84f2bf019e",
      "name": "hw01",
      "path": "CS 61A/practice/hw/hw01",
      "category": "practice",
      "course_code": "CS 61A"
    },
    {
      "module_uuid": "31578c41-f9e9-4e06-8b38-56b4a6e4eb62",
      "name": "lec01",
      "path": "CS 61A/study/lec01",
      "category": "study",
      "course_code": "CS 61A"
    }
  ],
  "total_count": 57,
  "course_code": "CS 61A"
}
```

**Response fields:**

| Field           | Type   | Description                                           |
|-----------------|--------|-------------------------------------------------------|
| `module_uuid`   | string | Stable UUID for this module. Use this in other APIs.  |
| `name`          | string | Short display name (last path segment): `hw01`, `lec01` |
| `path`          | string | Full directory path including course prefix            |
| `category`      | string | One of: `practice`, `study`, `support`                |
| `course_code`   | string | The course this module belongs to                     |

**Frontend usage notes:**
- Group modules by `category` to build a navigation tree
- Within `practice`, you can further group by the 3rd path segment (`hw`, `lab`, `proj`) parsed from `path`
- `module_uuid` is stable across server restarts — safe to persist in URLs and local storage
- `path` is for display only; always use `module_uuid` to identify modules in API calls

---

### List Files in Module

```
GET /api/modules/{module_uuid}/files
```

Returns all files belonging to a specific module. Supports pagination.

**Path Parameters:**

| Parameter      | Type   | Required | Description        |
|----------------|--------|----------|--------------------|
| `module_uuid`  | string | yes      | Module UUID        |

**Query Parameters:**

| Parameter | Type | Required | Default | Description      |
|-----------|------|----------|---------|------------------|
| `page`    | int  | no       | 1       | Page number      |
| `limit`   | int  | no       | 100     | Items per page (max 1000) |

**Response:** Same `FileListResponse` schema as the files API (see below).

**Error responses:**

| Status | Condition                     |
|--------|-------------------------------|
| 404    | Module UUID not found         |

---

## Files API

### List Files

```
GET /api/files
```

List files with filtering, search, and pagination.

**Query Parameters:**

| Parameter     | Type   | Required | Default | Description                               |
|---------------|--------|----------|---------|-------------------------------------------|
| `course_code` | string | no       | —       | Filter by course code                     |
| `category`    | string | no       | —       | Filter by category (document, video, audio, other) |
| `search`      | string | no       | —       | Search in file names and titles           |
| `path`        | string | no       | —       | Filter by directory path prefix           |
| `page`        | int    | no       | 1       | Page number                               |
| `limit`       | int    | no       | 100     | Items per page (max 1000)                 |

**Response (`FileListResponse`):**

```json
{
  "files": [
    {
      "uuid": "a1b2c3d4-...",
      "filename": "Homework 1 CS 61A Summer 2025.html",
      "title": "Homework 1 Cs 61A Summer 2025",
      "relative_path": "CS 61A/practice/hw/hw01/hw01/Homework 1 CS 61A Summer 2025.html",
      "size_bytes": 45231,
      "mime_type": "text/html",
      "created_at": null,
      "modified_at": null,
      "course": "CS 61A",
      "category": null,
      "sections": [
        {
          "name": "Required Questions",
          "index": 1.0,
          "key_concept": "Q1: A Plus Abs B",
          "aspects": [
            {
              "content": "Fill in the blanks...",
              "type": "Definition"
            }
          ],
          "checking_questions": null,
          "comprehensive_questions": null
        }
      ],
      "download_url": "/api/files/a1b2c3d4-.../download",
      "original_url": "https://cs61a.org/hw/hw01/"
    }
  ],
  "total_count": 42,
  "page": 1,
  "limit": 100,
  "has_next": false,
  "has_prev": false,
  "filters_applied": {
    "course_code": "CS 61A",
    "category": null,
    "search": null,
    "path": null
  }
}
```

**File fields:**

| Field           | Type             | Description                                      |
|-----------------|------------------|--------------------------------------------------|
| `uuid`          | string           | File UUID — use this to reference the file       |
| `filename`      | string           | Original filename                                |
| `title`         | string \| null   | Auto-generated display title                     |
| `relative_path` | string           | Full path in the course directory tree           |
| `size_bytes`    | int              | File size (0 if file not found on disk)          |
| `mime_type`     | string           | MIME type (e.g., `application/pdf`, `text/html`) |
| `course`        | string \| null   | Course code                                      |
| `sections`      | Section[]        | Parsed educational sections (see below)          |
| `download_url`  | string           | Relative URL to download this file               |
| `original_url`  | string \| null   | Source URL where the file was collected from      |

**Section schema:**

| Field                      | Type               | Description                          |
|----------------------------|---------------------|--------------------------------------|
| `name`                     | string              | Section title                        |
| `index`                    | float               | Section order/position               |
| `key_concept`              | string              | Main topic                           |
| `aspects`                  | SectionAspect[]     | List of content aspects              |
| `checking_questions`       | string[] \| null    | Quick comprehension questions        |
| `comprehensive_questions`  | string[] \| null    | Deeper review questions              |

---

### Browse Directory

```
GET /api/files/browse?course_code={course_code}&path={path}
```

Hierarchical directory browser. Returns immediate subdirectories and files at the given path.

**Query Parameters:**

| Parameter     | Type   | Required | Default | Description                                |
|---------------|--------|----------|---------|--------------------------------------------|
| `course_code` | string | yes      | —       | Course code                                |
| `path`        | string | no       | `""`    | Directory path within course (empty = root)|

**Response (`DirectoryBrowserResponse`):**

```json
{
  "directories": [
    {
      "name": "practice",
      "path": "CS 61A/practice",
      "file_count": 45,
      "has_subdirs": true
    }
  ],
  "files": [],
  "current_path": "",
  "breadcrumbs": [
    { "name": "Root", "path": "" }
  ],
  "course_code": "CS 61A"
}
```

---

### Get File Metadata

```
GET /api/files/{file_id}
```

Returns full metadata for a single file. Same `FileMetadata` schema as in list responses.

| Status | Condition              |
|--------|------------------------|
| 404    | File UUID not found    |

---

### Download File

```
GET /api/files/{file_id}/download
```

Returns the raw file content with correct MIME type and `Content-Disposition` headers.

| Status | Condition              |
|--------|------------------------|
| 404    | File UUID not found    |

---

### Get File Extra Info (Transcript)

```
GET /api/files/{file_id}/extra_info
```

Returns video transcript segments for files that have transcript data.

**Response:** `TranscriptSegment[]`

```json
[
  {
    "start_time": 0.0,
    "end_time": 15.5,
    "speaker": "instructor",
    "text_content": "Welcome to CS 61A..."
  }
]
```

Returns `[]` if no transcript data is available.

---

## Using Modules with Chat Completions

When sending a file-chat completion request (`POST /api/chat/completions` with `chat_type: "file"`), you can optionally include `module_uuid` in the `user_focus` object to restrict RAG retrieval to files within that module.

**Without `module_uuid`:** RAG searches across all files in the course.

**With `module_uuid`:** RAG only searches within files belonging to that module.

```json
{
  "course_code": "CS 61A",
  "chat_type": "file",
  "messages": [
    { "role": "user", "content": "Explain the map function from this lab" }
  ],
  "stream": true,
  "user_focus": {
    "file_uuid": "a1b2c3d4-...",
    "module_uuid": "79ada5b9-541c-4f03-a357-8c7228dc63aa"
  }
}
```

This is useful when the student is viewing a specific module (e.g., lab01) and you want the AI to prioritize references from that module's materials rather than pulling from unrelated parts of the course.

**Error responses:**

| Status | Condition                           |
|--------|-------------------------------------|
| 404    | `module_uuid` not found in database |
| 400    | Invalid request parameters          |

---

## Common Patterns

### Building a Module Navigation Sidebar

```
1. GET /api/modules?course_code=CS 61A
2. Group response by category (practice, study, support)
3. For practice modules, further group by path segment:
   - "CS 61A/practice/hw/hw01"  -> hw > hw01
   - "CS 61A/practice/lab/lab01" -> lab > lab01
   - "CS 61A/practice/proj/hog"  -> proj > hog
4. On click, fetch files:
   GET /api/modules/{module_uuid}/files
```

### Opening a File Chat with Module Scope

```
1. User selects a module (e.g., lab01)
2. Load files: GET /api/modules/{module_uuid}/files
3. User selects a file
4. Send chat with both file and module context:
   POST /api/chat/completions
   { user_focus: { file_uuid: "...", module_uuid: "..." } }
```
