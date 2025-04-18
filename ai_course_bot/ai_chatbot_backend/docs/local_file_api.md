# Local File Retrieval API Documentation

## Overview

The Local File Retrieval API provides endpoints to access, list, and manage files stored on the server's file system. This API is designed to be robust, extensible, and follows modern best practices for file organization and retrieval.

## Features

- List files with filtering by directory, category, and folder
- Retrieve files with proper mime-type detection for browser rendering
- Automatic file categorization based on file extension and path
- Support for hierarchical directory structure
- Course material organization with standard folders (Lab Material, Code Script, Exams, Past Projects)

## Directory Structure

Files are organized in a course-centric directory structure, where each course has its own set of directories:

```
data/
├── CS61A/
│   ├── documents/
│   │   ├── lab_material/
│   │   │   ├── 01_Getting_Started_Guide.pdf
│   │   │   └── 02_Lab_Instructions.pdf
│   │   ├── code_script/
│   │   │   └── example_code.py
│   │   ├── exams/
│   │   │   └── midterm1_study_guide.pdf
│   │   └── past_projects/
│   │       └── project1_hog.pdf
│   ├── videos/
│   │   └── 02_Lab_Instructions.mp4
│   ├── audios/
│   └── others/
├── CS61B/
│   ├── documents/
│   │   ├── lab_material/
│   │   ├── code_script/
│   │   ├── exams/
│   │   └── past_projects/
│   └── ...
└── CS170/
    └── ...
```

This structure makes it easy to:
1. Find all materials related to a specific course
2. Organize content by type within each course
3. Maintain consistent organization across different courses

## File Categorization

Files are automatically categorized based on their extension:

- **Document**: PDF, DOC, DOCX, TXT, MD
- **Assignment**: IPYNB, PY, JAVA, C, CPP, JS
- **Video**: MP4, AVI, MOV, MKV, WEBM
- **Others**: All other file types

## API Endpoints

### List Files

```
GET /v1/local-files
```

Lists all files in the system or in the specified directory.

**Query Parameters:**

- `directory` (optional): Directory to list files from (e.g., "CS61A/documents")
- `category` (optional): Filter files by category (Document, Assignment, Video, Others)
- `folder` (optional): Filter files by folder (Lab Material, Code Script, Exams, Past Projects)
- `course_code` (optional): Filter files by course code (e.g., "CS61A")
- `include_directories` (optional): Include directory information in response (default: false)
- `include_categories` (optional): Include category information in response (default: false)

**Example Request:**

```
GET /v1/local-files?directory=CS61A/documents&category=Document
```

**Example Response:**

```json
{
  "files": [
    {
      "file_name": "01_Getting_Started_Guide.pdf",
      "file_path": "CS61A/documents/lab_material/01_Getting_Started_Guide.pdf",
      "mime_type": "application/pdf",
      "size_bytes": 1048576,
      "modified_time": "2024-01-08T12:00:00",
      "directory": "CS61A/documents",
      "category": "Document",
      "folder": "Lab Material",
      "course_code": "CS61A"
    }
  ],
  "total_count": 1
}
```

### Get File

```
GET /v1/local-files/{file_path}
```

Retrieves a file by its path.

**Path Parameters:**

- `file_path`: Path to the file, relative to the base data directory

**Example Request:**

```
GET /v1/local-files/CS61A/documents/lab_material/01_Getting_Started_Guide.pdf
```

**Response:**

Returns the file content with appropriate Content-Type header.

### List Categories

```
GET /v1/local-files/categories
```

Lists all available file categories.

**Example Response:**

```json
[
  {
    "id": "document",
    "name": "Document",
    "icon": "file-text",
    "description": "Course documents and materials"
  },
  {
    "id": "assignment",
    "name": "Assignment",
    "icon": "clipboard",
    "description": "Homework and assignment files"
  },
  {
    "id": "video",
    "name": "Video",
    "icon": "video",
    "description": "Lecture videos and recordings"
  },
  {
    "id": "others",
    "name": "Others",
    "icon": "file",
    "description": "Other course-related files"
  }
]
```

### List Folders

```
GET /v1/local-files/folders
```

Lists all available file folders/directories.

**Example Response:**

```json
[
  {
    "name": "Lab Material",
    "path": "documents/lab_material",
    "is_category": true,
    "icon": "folder-laboratory"
  },
  {
    "name": "Code Script",
    "path": "documents/code_script",
    "is_category": true,
    "icon": "folder-code"
  },
  {
    "name": "Exams",
    "path": "documents/exams",
    "is_category": true,
    "icon": "folder-check"
  },
  {
    "name": "Past Projects",
    "path": "documents/past_projects",
    "is_category": true,
    "icon": "folder-archive"
  }
]
```

## Setting Up File Storage

### File Placement

To add files to be served by the API:

1. Run the initialization script to create the directory structure:
   ```bash
   python scripts/initialize_storage.py --clean --examples
   ```

2. Copy or save your files to the appropriate directory for each course
3. Name files with a consistent pattern for automatic metadata detection:
   - For lecture materials: `{number}_{title}.{extension}` (e.g., `01_Introduction.pdf`)
   - For assignments: `assignment{number}_{title}.{extension}` (e.g., `assignment1_recursion.pdf`)

### Example Organization for CS61A

```
data/
└── CS61A/
    ├── documents/
    │   ├── lab_material/
    │   │   ├── 01_Getting_Started_Guide.pdf
    │   │   └── 02_Lab_Instructions.pdf
    │   ├── code_script/
    │   │   └── example_code.py
    │   ├── exams/
    │   │   └── midterm1_study_guide.pdf
    │   └── past_projects/
    │       └── project1_hog.pdf
    └── videos/
        └── 02_Lab_Instructions.mp4
```

## Best Practices

1. **Course Organization**: Keep all files for a course within its directory
2. **File Naming**: Use a consistent naming pattern with numbering for ordered content
3. **Directory Structure**: Follow the standard directory structure for easy navigation
4. **File Types**: Use common file formats for maximum compatibility:
   - Documents: PDF for best compatibility
   - Code: Use plain text formats (PY, JS, etc.)
   - Videos: MP4 for best browser compatibility
5. **File Size**: Keep individual files under reasonable sizes (< 100MB) for smooth web delivery 