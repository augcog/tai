# Local File API Documentation

## Overview

The Local File API provides endpoints to access, list, and visualize files stored on the server's file system. This API is designed to be robust, extensible, and follows modern best practices for file organization and retrieval.

> **Important**: For authentication requirements and implementation details, please refer to the [Authentication Guide](authentication.md).
>
> **For Frontend Developers**: Comprehensive examples are available in the `postman/examples/local_files` directory, and an enhanced Postman collection can be generated using `scripts/enhanced_generate_local_file_postman_collection.py`.

## Features

- List files with filtering by directory, category, and folder
- Retrieve files with proper mime-type detection for browser rendering
- Automatic file categorization based on file extension and path
- Hierarchical file system visualization with customizable depth
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
Authorization: Bearer YOUR_AUTH_TOKEN
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
Authorization: Bearer YOUR_AUTH_TOKEN
```

**Alternative Request with Query Parameter Authentication:**

```
GET /v1/local-files/CS61A/documents/lab_material/01_Getting_Started_Guide.pdf?auth_token=YOUR_AUTH_TOKEN
```

**Response:**

Returns the file content with appropriate Content-Type header.

### Get File Hierarchy

```
GET /v1/local-files/hierarchy
```

Gets a hierarchical tree structure of files and directories.

**Query Parameters:**

- `directory` (optional): Directory to start from (e.g., "CS61A/documents")
- `max_depth` (optional): Maximum depth to traverse (-1 for unlimited)

**Example Request:**

```
GET /v1/local-files/hierarchy?directory=CS61A&max_depth=2
Authorization: Bearer YOUR_AUTH_TOKEN
```

**Example Response:**

```json
{
  "root": {
    "name": "CS61A",
    "path": "CS61A",
    "type": "directory",
    "children": [
      {
        "name": "documents",
        "path": "CS61A/documents",
        "type": "directory",
        "children": [
          {
            "name": "lab_material",
            "path": "CS61A/documents/lab_material",
            "type": "directory",
            "children": []
          },
          {
            "name": "code_script",
            "path": "CS61A/documents/code_script",
            "type": "directory",
            "children": []
          }
        ]
      },
      {
        "name": "videos",
        "path": "CS61A/videos",
        "type": "directory",
        "children": []
      }
    ]
  },
  "total_files": 0,
  "total_directories": 4,
  "max_depth": 2
}
```

### List Categories

```
GET /v1/local-files/categories
```

Lists all available file categories.

**Example Request:**

```
GET /v1/local-files/categories
Authorization: Bearer YOUR_AUTH_TOKEN
```

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

**Example Request:**

```
GET /v1/local-files/folders
Authorization: Bearer YOUR_AUTH_TOKEN
```

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

## Comprehensive Examples

To help frontend developers understand how the API behaves with different parameter combinations, we provide a set of comprehensive examples in the `postman/examples/local_files` directory.

### Example Structure

```
postman/examples/local_files/
├── README.md                                # Overview of examples
├── list_files/                              # Examples for the List Files endpoint
│   ├── basic_list.json                      # Basic listing without filters
│   ├── filtered_by_directory.json           # Filtering by directory
│   ├── filtered_by_category.json            # Filtering by category
│   ├── filtered_by_folder.json              # Filtering by folder
│   ├── filtered_by_course_code.json         # Filtering by course code
│   ├── with_directories_and_categories.json # Including directory and category info
│   ├── empty_directory.json                 # Listing an empty directory
│   └── multiple_filters.json                # Using multiple filters together
├── file_hierarchy/                          # Examples for the File Hierarchy endpoint
│   ├── root_hierarchy.json                  # Full hierarchy from root
│   ├── specific_directory.json              # Hierarchy from a specific directory
│   ├── limited_depth.json                   # Hierarchy with limited depth
│   └── empty_directory.json                 # Hierarchy for an empty directory
└── categories_folders/                      # Examples for Categories and Folders endpoints
    ├── categories.json                      # List of file categories
    └── folders.json                         # List of file folders
```

Each example JSON file follows this format:

```json
{
  "meta": {
    "name": "Example Name",
    "description": "Description of the example",
    "tags": ["local_files", "tag1", "tag2"]
  },
  "request": {
    "endpoint": "GET /v1/local-files",
    "parameters": {
      "param1": "value1",
      "param2": "value2"
    }
  },
  "response": {
    // Example response JSON
  }
}
```

### Important Parameter Combinations

#### List Files Endpoint

- **Basic listing**: No parameters
- **Directory filtering**: `directory=documents/CS61A`
- **Category filtering**: `category=Document`
- **Folder filtering**: `folder=Lab Material`
- **Course filtering**: `course_code=CS61A`
- **Including metadata**: `include_directories=true&include_categories=true`
- **Multiple filters**: Combining directory, category, folder, and course filters

#### File Hierarchy Endpoint

- **Full hierarchy**: No parameters
- **Specific directory**: `directory=documents/CS61A`
- **Limited depth**: `max_depth=1` or `max_depth=2`
- **Empty directory**: Directory with no files or subdirectories

### Edge Cases

The examples also cover important edge cases:

- Empty directories
- Deep nested directories
- Various file types (PDF, Python, MP4, etc.)
- Files with different metadata attributes

## Postman Collections

Two Postman collections are available for testing the Local File API:

1. **Basic Collection** (`postman/local_file_postman_collection.json`):
   - Simple collection with basic examples
   - Provides essential requests for each endpoint

2. **Enhanced Collection** (`postman/enhanced_local_file_postman_collection.json`):
   - Comprehensive collection with multiple examples for each endpoint
   - Demonstrates various parameter combinations and edge cases
   - Includes better organization with requests grouped by functionality
   - Recommended for frontend development

### Generating the Postman Collections

To generate the basic Postman collection:

```bash
python scripts/generate_local_file_postman_collection.py --output postman/local_file_postman_collection.json
```

To generate the enhanced Postman collection:

```bash
python scripts/enhanced_generate_local_file_postman_collection.py
```

This will create the collections in the `postman/` directory that you can import into Postman.

## Best Practices

1. **Course Organization**: Keep all files for a course within its directory
2. **File Naming**: Use a consistent naming pattern with numbering for ordered content
3. **Directory Structure**: Follow the standard directory structure for easy navigation
4. **File Types**: Use common file formats for maximum compatibility:
   - Documents: PDF for best compatibility
   - Code: Use plain text formats (PY, JS, etc.)
   - Videos: MP4 for best browser compatibility
5. **File Size**: Keep individual files under reasonable sizes (< 100MB) for smooth web delivery
6. **Hierarchy Requests**:
   - Use the `max_depth` parameter to limit the depth of the hierarchy for better performance
   - For large directories, consider starting from a subdirectory rather than the root
   - Cache hierarchy responses on the client side when appropriate
7. **Authentication**:
   - Always include authentication with API requests
   - Use header-based authentication (`Authorization: Bearer TOKEN`) for programmatic access
   - Use query parameter authentication (`?auth_token=TOKEN`) for embedded resources like images
   - See the [Authentication Guide](authentication.md) for detailed implementation instructions
8. **Frontend Integration**:
   - Use the comprehensive examples to understand API behavior
   - Test with the enhanced Postman collection before implementing
   - Handle all parameter combinations and edge cases
   - Implement proper error handling for missing files or directories