#!/usr/bin/env python3
"""
Enhanced Local File API Postman Collection Generator

This script creates a comprehensive Postman collection for the Local File API endpoints
with multiple examples for each endpoint to demonstrate different parameter combinations.

Usage:
    python enhanced_generate_local_file_postman_collection.py [--output FILENAME] [--examples-dir DIRECTORY]

Options:
    --output FILENAME       Specify the output filename (default: enhanced_local_file_postman_collection.json)
    --examples-dir DIR      Directory containing example JSON files (default: examples/local_files)
"""

import os
import json
import argparse
import glob
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional

DEFAULT_OUTPUT = "postman/enhanced_local_file_postman_collection.json"
DEFAULT_EXAMPLES_DIR = "postman/examples/local_files"
API_BASE_URL = "{{apiBaseUrl}}"  # Postman variable for API base URL

def load_example_json(file_path: str) -> Dict:
    """Load an example JSON file."""
    try:
        with open(file_path, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading example file {file_path}: {str(e)}")
        return {}

def load_examples_from_directory(directory: str, subdirectory: Optional[str] = None) -> List[Dict]:
    """Load all example JSON files from a directory."""
    examples = []

    # Construct the path to the examples
    path = Path(directory)
    if subdirectory:
        path = path / subdirectory

    # Find all JSON files in the directory
    json_files = glob.glob(str(path / "*.json"))

    for file_path in json_files:
        example = load_example_json(file_path)
        if example:
            examples.append(example)

    return examples

def create_list_files_requests(examples_dir: str) -> List[Dict]:
    """Create Postman requests for the List Files endpoint with multiple examples."""
    list_files_examples = load_examples_from_directory(examples_dir, "list_files")
    requests = []

    # Basic List Files request
    basic_list_request = {
        "name": "List Files (Basic)",
        "description": "List all files without any filters",
        "request": {
            "method": "GET",
            "header": [
                {
                    "key": "Authorization",
                    "value": "Bearer {{authToken}}"
                }
            ],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files"
                ]
            }
        },
        "response": []
    }

    # Add example responses
    for example in list_files_examples:
        if "meta" in example and "request" in example and "response" in example:
            meta = example["meta"]
            request_data = example["request"]
            response_data = example["response"]

            # Skip if not relevant to this endpoint
            if "list" not in meta.get("tags", []):
                continue

            # Create response object
            response_object = {
                "name": meta.get("name", "Example Response"),
                "originalRequest": {
                    "method": "GET",
                    "header": [
                        {
                            "key": "Authorization",
                            "value": "Bearer {{authToken}}"
                        }
                    ],
                    "url": {
                        "raw": f"{API_BASE_URL}/v1/local-files",
                        "host": [
                            "{{apiBaseUrl}}"
                        ],
                        "path": [
                            "v1",
                            "local-files"
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
                "body": json.dumps(response_data, indent=2),
                "_postman_previewlanguage": "json"
            }

            # Add query parameters if present
            if "parameters" in request_data and request_data["parameters"]:
                query_params = []
                url_raw = f"{API_BASE_URL}/v1/local-files"

                for key, value in request_data["parameters"].items():
                    query_params.append({
                        "key": key,
                        "value": str(value),
                        "description": get_parameter_description(key)
                    })
                    url_raw += f"{'?' if len(query_params) == 1 else '&'}{key}={value}"

                response_object["originalRequest"]["url"]["query"] = query_params
                response_object["originalRequest"]["url"]["raw"] = url_raw

            # Add to the appropriate request based on tags
            if "basic" in meta.get("tags", []):
                basic_list_request["response"].append(response_object)
            else:
                # Create a specific request for this example
                specific_request = {
                    "name": meta.get("name", "List Files Example"),
                    "description": meta.get("description", ""),
                    "request": {
                        "method": "GET",
                        "header": [
                            {
                                "key": "Authorization",
                                "value": "Bearer {{authToken}}"
                            }
                        ],
                        "url": {
                            "raw": f"{API_BASE_URL}/v1/local-files",
                            "host": [
                                "{{apiBaseUrl}}"
                            ],
                            "path": [
                                "v1",
                                "local-files"
                            ]
                        }
                    },
                    "response": [response_object]
                }

                # Add query parameters if present
                if "parameters" in request_data and request_data["parameters"]:
                    query_params = []
                    url_raw = f"{API_BASE_URL}/v1/local-files"

                    for key, value in request_data["parameters"].items():
                        query_params.append({
                            "key": key,
                            "value": str(value),
                            "description": get_parameter_description(key)
                        })
                        url_raw += f"{'?' if len(query_params) == 1 else '&'}{key}={value}"

                    specific_request["request"]["url"]["query"] = query_params
                    specific_request["request"]["url"]["raw"] = url_raw

                requests.append(specific_request)

    # Add the basic request first
    requests.insert(0, basic_list_request)

    return requests

def create_file_hierarchy_requests(examples_dir: str) -> List[Dict]:
    """Create Postman requests for the File Hierarchy endpoint with multiple examples."""
    hierarchy_examples = load_examples_from_directory(examples_dir, "file_hierarchy")
    requests = []

    # Basic Hierarchy request
    basic_hierarchy_request = {
        "name": "Get File Hierarchy (Root)",
        "description": "Get a hierarchical tree structure of files and directories from the root",
        "request": {
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
        "response": []
    }

    # Add example responses
    for example in hierarchy_examples:
        if "meta" in example and "request" in example and "response" in example:
            meta = example["meta"]
            request_data = example["request"]
            response_data = example["response"]

            # Skip if not relevant to this endpoint
            if "hierarchy" not in meta.get("tags", []):
                continue

            # Create response object
            response_object = {
                "name": meta.get("name", "Example Response"),
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
                "body": json.dumps(response_data, indent=2),
                "_postman_previewlanguage": "json"
            }

            # Add query parameters if present
            if "parameters" in request_data and request_data["parameters"]:
                query_params = []
                url_raw = f"{API_BASE_URL}/v1/local-files/hierarchy"

                for key, value in request_data["parameters"].items():
                    query_params.append({
                        "key": key,
                        "value": str(value),
                        "description": get_parameter_description(key)
                    })
                    url_raw += f"{'?' if len(query_params) == 1 else '&'}{key}={value}"

                response_object["originalRequest"]["url"]["query"] = query_params
                response_object["originalRequest"]["url"]["raw"] = url_raw

            # Add to the appropriate request based on tags
            if "root" in meta.get("tags", []):
                basic_hierarchy_request["response"].append(response_object)
            else:
                # Create a specific request for this example
                specific_request = {
                    "name": meta.get("name", "Get File Hierarchy Example"),
                    "description": meta.get("description", ""),
                    "request": {
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
                    "response": [response_object]
                }

                # Add query parameters if present
                if "parameters" in request_data and request_data["parameters"]:
                    query_params = []
                    url_raw = f"{API_BASE_URL}/v1/local-files/hierarchy"

                    for key, value in request_data["parameters"].items():
                        query_params.append({
                            "key": key,
                            "value": str(value),
                            "description": get_parameter_description(key)
                        })
                        url_raw += f"{'?' if len(query_params) == 1 else '&'}{key}={value}"

                    specific_request["request"]["url"]["query"] = query_params
                    specific_request["request"]["url"]["raw"] = url_raw

                requests.append(specific_request)

    # Add the basic request first
    requests.insert(0, basic_hierarchy_request)

    return requests

def create_categories_folders_requests(examples_dir: str) -> List[Dict]:
    """Create Postman requests for the Categories and Folders endpoints."""
    examples = load_examples_from_directory(examples_dir, "categories_folders")
    requests = []

    # Categories request
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
        "response": []
    }

    # Folders request
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
        "response": []
    }

    # Add example responses
    for example in examples:
        if "meta" in example and "request" in example and "response" in example:
            meta = example["meta"]
            request_data = example["request"]
            response_data = example["response"]

            # Create response object
            response_object = {
                "name": meta.get("name", "Example Response"),
                "originalRequest": {
                    "method": "GET",
                    "header": [
                        {
                            "key": "Authorization",
                            "value": "Bearer {{authToken}}"
                        }
                    ],
                    "url": {
                        "raw": "",
                        "host": [
                            "{{apiBaseUrl}}"
                        ],
                        "path": [
                            "v1",
                            "local-files"
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
                "body": json.dumps(response_data, indent=2),
                "_postman_previewlanguage": "json"
            }

            # Add to the appropriate request based on tags
            if "categories" in meta.get("tags", []):
                response_object["originalRequest"]["url"]["raw"] = f"{API_BASE_URL}/v1/local-files/categories"
                response_object["originalRequest"]["url"]["path"].append("categories")
                categories_request["response"].append(response_object)
            elif "folders" in meta.get("tags", []):
                response_object["originalRequest"]["url"]["raw"] = f"{API_BASE_URL}/v1/local-files/folders"
                response_object["originalRequest"]["url"]["path"].append("folders")
                folders_request["response"].append(response_object)

    requests.append(categories_request)
    requests.append(folders_request)

    return requests

def create_get_file_requests() -> List[Dict]:
    """Create Postman requests for the Get File endpoint."""
    requests = []

    # Get File request
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
                "raw": f"{API_BASE_URL}/v1/local-files/CS61A/documents/lab_material/01_Getting_Started_Guide.txt",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "CS61A",
                    "documents",
                    "lab_material",
                    "01_Getting_Started_Guide.txt"
                ]
            }
        },
        "response": []
    }

    # Get File with Query Auth request
    get_file_query_auth_request = {
        "name": "Get File (Query Auth)",
        "description": "Retrieve a file by its path using query parameter authentication",
        "request": {
            "method": "GET",
            "header": [],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files/CS61A/documents/lab_material/01_Getting_Started_Guide.txt?auth_token={{authToken}}",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "CS61A",
                    "documents",
                    "lab_material",
                    "01_Getting_Started_Guide.txt"
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

    # Get PDF File request
    get_pdf_file_request = {
        "name": "Get PDF File",
        "description": "Retrieve a PDF file by its path",
        "request": {
            "method": "GET",
            "header": [
                {
                    "key": "Authorization",
                    "value": "Bearer {{authToken}}"
                }
            ],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files/CS61A/documents/exams/midterm_sample.pdf",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "CS61A",
                    "documents",
                    "exams",
                    "midterm_sample.pdf"
                ]
            }
        },
        "response": []
    }

    # Get Python File request
    get_python_file_request = {
        "name": "Get Python File",
        "description": "Retrieve a Python file by its path",
        "request": {
            "method": "GET",
            "header": [
                {
                    "key": "Authorization",
                    "value": "Bearer {{authToken}}"
                }
            ],
            "url": {
                "raw": f"{API_BASE_URL}/v1/local-files/CS61A/documents/code_script/example_code.py",
                "host": [
                    "{{apiBaseUrl}}"
                ],
                "path": [
                    "v1",
                    "local-files",
                    "CS61A",
                    "documents",
                    "code_script",
                    "example_code.py"
                ]
            }
        },
        "response": []
    }

    requests.append(get_file_request)
    requests.append(get_file_query_auth_request)
    requests.append(get_pdf_file_request)
    requests.append(get_python_file_request)

    return requests

def get_parameter_description(param_name: str) -> str:
    """Get a description for a parameter."""
    descriptions = {
        "directory": "Directory to list files from or start hierarchy from",
        "category": "Filter files by category (Document, Assignment, Video, Others)",
        "folder": "Filter files by folder (Lab Material, Code Script, Exams, Past Projects)",
        "course_code": "Filter files by course code",
        "include_directories": "Include directory information in response",
        "include_categories": "Include category information in response",
        "max_depth": "Maximum depth to traverse in hierarchy (-1 for unlimited)",
        "auth_token": "Authentication token for query parameter authentication"
    }

    return descriptions.get(param_name, "")

def create_enhanced_local_file_postman_collection(examples_dir: str) -> Dict:
    """Create an enhanced Postman collection for the Local File API."""
    collection = {
        "info": {
            "name": "AI Chatbot Local File API (Enhanced)",
            "description": "Comprehensive collection for testing the AI Chatbot Local File API endpoints with multiple examples",
            "schema": "https://schema.getpostman.com/json/collection/v2.1.0/collection.json",
            "_postman_id": f"ai-chatbot-local-file-enhanced-{datetime.now().strftime('%Y%m%d%H%M%S')}",
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

    list_files_folder = {
        "name": "List Files",
        "description": "Endpoints for listing files with various filters",
        "item": create_list_files_requests(examples_dir)
    }

    get_file_folder = {
        "name": "Get File",
        "description": "Endpoints for retrieving individual files",
        "item": create_get_file_requests()
    }

    hierarchy_folder = {
        "name": "File Hierarchy",
        "description": "File hierarchy visualization endpoints",
        "item": create_file_hierarchy_requests(examples_dir)
    }

    metadata_folder = {
        "name": "Metadata",
        "description": "Endpoints for file metadata like categories and folders",
        "item": create_categories_folders_requests(examples_dir)
    }

    # Add subfolders to the file operations folder
    file_operations_folder["item"].append(list_files_folder)
    file_operations_folder["item"].append(get_file_folder)

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
    print(f"Enhanced Local File API Postman collection saved to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Generate an enhanced Postman collection for the Local File API")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"Output filename (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--examples-dir", default=DEFAULT_EXAMPLES_DIR, help=f"Directory containing example JSON files (default: {DEFAULT_EXAMPLES_DIR})")
    args = parser.parse_args()

    collection = create_enhanced_local_file_postman_collection(args.examples_dir)
    save_collection(collection, args.output)

if __name__ == "__main__":
    main()
