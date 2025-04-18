#!/usr/bin/env python3
"""
Postman Collection Generator

This script creates a Postman collection from the fixtures in the 
AI chatbot backend API. The collection can be imported into Postman
for easy testing.

Usage:
    python generate_postman_collection.py [--output FILENAME]

Options:
    --output FILENAME    Specify the output filename (default: postman_collection.json)
"""

import os
import json
import argparse
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any

FIXTURES_DIR = Path("ai_course_bot/ai_chatbot_backend/tests/fixtures")
DEFAULT_OUTPUT = "postman_collection.json"
API_BASE_URL = "{{apiBaseUrl}}"  # Postman variable for API base URL

def load_fixtures(directory: Path) -> Dict[str, List[Dict]]:
    """Load all fixture files from the given directory."""
    fixtures = {
        "requests": [],
        "responses": []
    }
    
    request_dir = directory / "endpoints" / "completions" / "requests"
    response_dir = directory / "endpoints" / "completions" / "responses"
    
    if request_dir.exists():
        for file in request_dir.glob("*.json"):
            with open(file, "r") as f:
                fixture = json.load(f)
                fixture["_filename"] = file.name
                fixtures["requests"].append(fixture)
    
    if response_dir.exists():
        for file in response_dir.glob("*.json"):
            with open(file, "r") as f:
                fixture = json.load(f)
                fixture["_filename"] = file.name
                fixtures["responses"].append(fixture)
    
    return fixtures

def create_postman_collection(fixtures: Dict[str, List[Dict]]) -> Dict:
    """Create a Postman collection from fixtures."""
    collection = {
        "info": {
            "name": "AI Chatbot API",
            "description": "Collection for testing the AI Chatbot backend API",
            "schema": "https://schema.getpostman.com/json/collection/v2.1.0/collection.json",
            "_postman_id": f"ai-chatbot-{datetime.now().strftime('%Y%m%d%H%M%S')}",
            "version": "1.0.0"
        },
        "variable": [
            {
                "key": "apiBaseUrl",
                "value": "http://localhost:8000",
                "type": "string"
            }
        ],
        "item": []
    }
    
    # Create folder for completions endpoint
    completions_folder = {
        "name": "Completions",
        "description": "Endpoints for chat completions",
        "item": []
    }
    
    # Create request items from fixtures
    for request_fixture in fixtures["requests"]:
        meta = request_fixture.get("meta", {})
        data = request_fixture.get("data", {})
        filename = request_fixture.get("_filename", "unknown.json")
        
        # Skip if there's no data
        if not data:
            continue
        
        # Extract information
        name = meta.get("name", "Unknown Request")
        description = meta.get("description", "")
        tags = meta.get("tags", [])
        
        # Determine if streaming
        is_streaming = "streaming" in tags or data.get("stream", False)
        
        # Determine if RAG
        is_rag = "rag" in tags or data.get("rag", False)
        
        # Create Postman request
        request_item = {
            "name": name,
            "description": description + f"\n\nSource: {filename}",
            "request": {
                "method": "POST",
                "header": [
                    {
                        "key": "Content-Type",
                        "value": "application/json"
                    }
                ],
                "url": {
                    "raw": f"{API_BASE_URL}/v1/completions",
                    "host": [
                        "{{apiBaseUrl}}"
                    ],
                    "path": [
                        "v1",
                        "completions"
                    ]
                },
                "body": {
                    "mode": "raw",
                    "raw": json.dumps(data, indent=2),
                    "options": {
                        "raw": {
                            "language": "json"
                        }
                    }
                }
            },
            "response": []
        }
        
        # Add relevant responses as examples
        for response_fixture in fixtures["responses"]:
            response_meta = response_fixture.get("meta", {})
            response_data = response_fixture.get("data", {})
            response_filename = response_fixture.get("_filename", "unknown.json")
            
            # Skip if there's no data
            if not response_data:
                continue
            
            response_name = response_meta.get("name", "Unknown Response")
            response_tags = response_meta.get("tags", [])
            
            # Match appropriate responses based on tags
            streaming_match = ("streaming" in response_tags) == is_streaming
            rag_match = (("rag" in response_tags) == is_rag) or ("references" in response_tags and is_rag)
            
            if streaming_match and rag_match:
                # Create Postman response
                response_object = {
                    "name": response_name,
                    "originalRequest": request_item["request"],
                    "status": "OK",
                    "code": 200,
                    "header": [
                        {
                            "key": "Content-Type",
                            "value": "application/json" if not is_streaming else "text/event-stream"
                        }
                    ],
                    "body": json.dumps(response_data, indent=2),
                    "_postman_previewlanguage": "json"
                }
                
                request_item["response"].append(response_object)
        
        completions_folder["item"].append(request_item)
    
    collection["item"].append(completions_folder)
    return collection

def save_collection(collection: Dict, output_path: str) -> None:
    """Save the Postman collection to a file."""
    with open(output_path, "w") as f:
        json.dump(collection, f, indent=2)
    print(f"Postman collection saved to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Generate a Postman collection from fixtures")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"Output filename (default: {DEFAULT_OUTPUT})")
    args = parser.parse_args()
    
    fixtures = load_fixtures(FIXTURES_DIR)
    collection = create_postman_collection(fixtures)
    save_collection(collection, args.output)

if __name__ == "__main__":
    main() 