#!/usr/bin/env python3
"""
Local File API Postman Collection Generator

This script creates a Postman collection for the Local File API endpoints.
The collection can be imported into Postman for easy testing.

Usage:
    python generate_local_file_postman_collection.py [--output FILENAME]

Options:
    --output FILENAME    Specify the output filename (default: local_file_postman_collection.json)
"""

import os
import json
import argparse
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any

DEFAULT_OUTPUT = "local_file_postman_collection.json"
API_BASE_URL = "{{apiBaseUrl}}"  # Postman variable for API base URL

def create_local_file_postman_collection() -> Dict:
    """Create a Postman collection for the Local File API."""
    collection = {
        "info": {
            "name": "AI Chatbot Local File API",
            "description": "Collection for testing the AI Chatbot Local File API endpoints",
            "schema": "https://schema.getpostman.com/json/collection/v2.1.0/collection.json",
            "_postman_id": f"ai-chatbot-local-file-{datetime.now().strftime('%Y%m%d%H%M%S')}",
            "version": "1.0.0"
        },
        "variable": [
            {
                "key": "apiBaseUrl",
                "value": "http://localhost:8000",
                "type": "string"
            },
            {
                "key": "authToken",
                "value": "YOUR_AUTH_TOKEN",
                "type": "string"
            }
        ],
        "item": []
    }

    # Create folder for local-files endpoint
    local_files_folder = {
        "name": "Local Files",
        "description": "Endpoints for accessing and managing local files",
        "item": []
    }

    # Create subfolders for better organization
    file_operations_folder = {
        "name": "File Operations",
        "description": "Basic file operations like listing and retrieving files",
        "item": []
    }

    hierarchy_folder = {
        "name": "Hierarchy",
        "description": "File hierarchy visualization endpoints",
        "item": []
    }

    metadata_folder = {
        "name": "Metadata",
        "description": "Endpoints for file metadata like categories and folders",
        "item": []
    }

    # Add List Files request
    list_files_request = {
        "name": "List Files",
        "description": "List files in a directory with optional filtering",
        "request": {
            "method": "GET",
            "header": [
                {
                    "key": "Authorization",
                    "value": "Bearer {{authToken}}"
                }
            ],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files?directory=&category=&folder=&course_code=&include_directories=true&include_categories=true",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files"
                ],
                "query": [
                    {
                        "key": "directory",
                        "value": "",
                        "description": "Directory to list files from"
                    },
                    {
                        "key": "category",
                        "value": "",
                        "description": "Filter files by category (Document, Assignment, Video, Others)"
                    },
                    {
                        "key": "folder",
                        "value": "",
                        "description": "Filter files by folder (Lab Material, Code Script, Exams, Past Projects)"
                    },
                    {
                        "key": "course_code",
                        "value": "",
                        "description": "Filter files by course code"
                    },
                    {
                        "key": "include_directories",
                        "value": "true",
                        "description": "Include directory information in response"
                    },
                    {
                        "key": "include_categories",
                        "value": "true",
                        "description": "Include category information in response"
                    }
                ]
            }
        },
        "response": [
            {
                "name": "Example Response",
                "originalRequest": {
                    "method": "GET",
                    "header": [
                        {
                            "key": "Authorization",
                            "value": "Bearer {{authToken}}"
                        }
                    ],
                    "url": {
                        "raw": f"{API_BASE_URL}/v1/local-files?directory=&include_directories=true&include_categories=true",
                        "host": [
                            "{{apiBaseUrl}}"
                        ],
                        "path": [
                            "v1",
                            "local-files"
                        ],
                        "query": [
                            {
                                "key": "directory",
                                "value": ""
                            },
                            {
                                "key": "include_directories",
                                "value": "true"
                            },
                            {
                                "key": "include_categories",
                                "value": "true"
                            }
                        ]
                    }
                },
                "status": "OK",
                "code": 200,
                "header": [
                    {
                        "key": "Content-Type",
                        "value": "application/json"
                    }
                ],
                "body": json.dumps({
                    "files": [
                        {
                            "file_name": "lecture1.pdf",
                            "file_path": "documents/CS61A/lecture1.pdf",
                            "mime_type": "application/pdf",
                            "size_bytes": 1048576,
                            "modified_time": "2023-01-01T12:00:00",
                            "directory": "documents",
                            "category": "Document",
                            "folder": "Lab Material"
                        }
                    ],
                    "total_count": 1,
                    "directories": [
                        {
                            "name": "Lab Material",
                            "path": "documents/CS61A/lab",
                            "parent_path": "documents/CS61A",
                            "is_category": False,
                            "icon": "folder"
                        }
                    ],
                    "categories": [
                        {
                            "id": "document",
                            "name": "Document",
                            "icon": "file-text",
                            "description": "Course documents and materials"
                        }
                    ]
                }, indent=2),
                "_postman_previewlanguage": "json"
            }
        ]
    }

    # Add Get File request
    get_file_request = {
        "name": "Get File",
        "description": "Retrieve a file by its path",
        "request": {
            "method": "GET",
            "header": [
                {
                    "key": "Authorization",
                    "value": "Bearer {{authToken}}"
                }
            ],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files/documents/example.pdf",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "documents",
                    "example.pdf"
                ]
            }
        },
        "response": []
    }

    # Add Get File with Query Auth request
    get_file_query_auth_request = {
        "name": "Get File (Query Auth)",
        "description": "Retrieve a file by its path using query parameter authentication",
        "request": {
            "method": "GET",
            "header": [],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files/documents/example.pdf?auth_token={{authToken}}",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "documents",
                    "example.pdf"
                ],
                "query": [
                    {
                        "key": "auth_token",
                        "value": "{{authToken}}",
                        "description": "Authentication token"
                    }
                ]
            }
        },
        "response": []
    }

    # Add Get File Hierarchy request
    get_hierarchy_request = {
        "name": "Get File Hierarchy",
        "description": "Get a hierarchical tree structure of files and directories",
        "request": {
            "method": "GET",
            "header": [
                {
                    "key": "Authorization",
                    "value": "Bearer {{authToken}}"
                }
            ],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files/hierarchy?directory=&max_depth=-1",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "hierarchy"
                ],
                "query": [
                    {
                        "key": "directory",
                        "value": "",
                        "description": "Directory to start from (optional)"
                    },
                    {
                        "key": "max_depth",
                        "value": "-1",
                        "description": "Maximum depth to traverse (-1 for unlimited)"
                    }
                ]
            }
        },
        "response": [
            {
                "name": "Example Response",
                "originalRequest": {
                    "method": "GET",
                    "header": [
                        {
                            "key": "Authorization",
                            "value": "Bearer {{authToken}}"
                        }
                    ],
                    "url": {
                        "raw": f"{API_BASE_URL}/v1/local-files/hierarchy",
                        "host": [
                            "{{apiBaseUrl}}"
                        ],
                        "path": [
                            "v1",
                            "local-files",
                            "hierarchy"
                        ]
                    }
                },
                "status": "OK",
                "code": 200,
                "header": [
                    {
                        "key": "Content-Type",
                        "value": "application/json"
                    }
                ],
                "body": json.dumps({
                    "root": {
                        "name": "root",
                        "path": "",
                        "type": "directory",
                        "children": [
                            {
                                "name": "documents",
                                "path": "documents",
                                "type": "directory",
                                "children": [
                                    {
                                        "name": "lecture1.pdf",
                                        "path": "documents/lecture1.pdf",
                                        "type": "file",
                                        "mime_type": "application/pdf",
                                        "size_bytes": 1048576,
                                        "modified_time": "2023-01-01T12:00:00"
                                    }
                                ]
                            }
                        ]
                    },
                    "total_files": 1,
                    "total_directories": 2,
                    "max_depth": 2
                }, indent=2),
                "_postman_previewlanguage": "json"
            }
        ]
    }

    # Add requests to the appropriate subfolders
    file_operations_folder["item"].append(list_files_request)
    file_operations_folder["item"].append(get_file_request)
    file_operations_folder["item"].append(get_file_query_auth_request)

    hierarchy_folder["item"].append(get_hierarchy_request)

    # Add categories request
    categories_request = {
        "name": "List Categories",
        "description": "Lists all available file categories",
        "request": {
            "method": "GET",
            "header": [
                {
                    "key": "Authorization",
                    "value": "Bearer {{authToken}}"
                }
            ],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files/categories",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "categories"
                ]
            }
        },
        "response": [
            {
                "name": "Example Response",
                "originalRequest": {
                    "method": "GET",
                    "header": [
                        {
                            "key": "Authorization",
                            "value": "Bearer {{authToken}}"
                        }
                    ],
                    "url": {
                        "raw": f"{API_BASE_URL}/v1/local-files/categories",
                        "host": [
                            "{{apiBaseUrl}}"
                        ],
                        "path": [
                            "v1",
                            "local-files",
                            "categories"
                        ]
                    }
                },
                "status": "OK",
                "code": 200,
                "header": [
                    {
                        "key": "Content-Type",
                        "value": "application/json"
                    }
                ],
                "body": json.dumps([
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
                ], indent=2),
                "_postman_previewlanguage": "json"
            }
        ]
    }

    # Add folders request
    folders_request = {
        "name": "List Folders",
        "description": "Lists all available file folders/directories",
        "request": {
            "method": "GET",
            "header": [
                {
                    "key": "Authorization",
                    "value": "Bearer {{authToken}}"
                }
            ],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files/folders",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "folders"
                ]
            }
        },
        "response": [
            {
                "name": "Example Response",
                "originalRequest": {
                    "method": "GET",
                    "header": [
                        {
                            "key": "Authorization",
                            "value": "Bearer {{authToken}}"
                        }
                    ],
                    "url": {
                        "raw": f"{API_BASE_URL}/v1/local-files/folders",
                        "host": [
                            "{{apiBaseUrl}}"
                        ],
                        "path": [
                            "v1",
                            "local-files",
                            "folders"
                        ]
                    }
                },
                "status": "OK",
                "code": 200,
                "header": [
                    {
                        "key": "Content-Type",
                        "value": "application/json"
                    }
                ],
                "body": json.dumps([
                    {
                        "name": "Lab Material",
                        "path": "documents/lab_material",
                        "is_category": True,
                        "icon": "folder-laboratory"
                    },
                    {
                        "name": "Code Script",
                        "path": "documents/code_script",
                        "is_category": True,
                        "icon": "folder-code"
                    },
                    {
                        "name": "Exams",
                        "path": "documents/exams",
                        "is_category": True,
                        "icon": "folder-check"
                    },
                    {
                        "name": "Past Projects",
                        "path": "documents/past_projects",
                        "is_category": True,
                        "icon": "folder-archive"
                    }
                ], indent=2),
                "_postman_previewlanguage": "json"
            }
        ]
    }

    metadata_folder["item"].append(categories_request)
    metadata_folder["item"].append(folders_request)

    # Add subfolders to the main folder
    local_files_folder["item"].append(file_operations_folder)
    local_files_folder["item"].append(hierarchy_folder)
    local_files_folder["item"].append(metadata_folder)

    # Add the main folder to the collection
    collection["item"].append(local_files_folder)

    return collection

def save_collection(collection: Dict, output_path: str) -> None:
    """Save the Postman collection to a file."""
    with open(output_path, "w") as f:
        json.dump(collection, f, indent=2)
    print(f"Local File API Postman collection saved to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Generate a Postman collection for the Local File API")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"Output filename (default: {DEFAULT_OUTPUT})")
    args = parser.parse_args()

    collection = create_local_file_postman_collection()
    save_collection(collection, args.output)

if __name__ == "__main__":
    main()
