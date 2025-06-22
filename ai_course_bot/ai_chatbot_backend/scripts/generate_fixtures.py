#!/usr/bin/env python3
"""
Fixture Generator Script

This script generates test fixtures for the AI chatbot backend API. It creates
JSON files for requests and responses with a flexible template system to avoid
repetition and ensure consistency.

Usage:
    python generate_fixtures.py [--clean]

Options:
    --clean     Remove existing fixture files before generating new ones

The script will generate all fixtures in the appropriate directories.
"""

import json
import argparse
from pathlib import Path
from typing import Dict, List, Any, Optional, Union

FIXTURES_DIR = Path("ai_course_bot/ai_chatbot_backend/tests/fixtures")

# Template definitions for common patterns
TEMPLATES = {
    "base_meta": {
        "name": "",
        "description": "",
        "tags": []
    },
    "base_request": {
        "model": "custom-model",
        "messages": [],
        "temperature": 0.7,
        "max_tokens": 150
    },
    "base_response": {
        "id": "chatcmpl-123",
        "created": 1677652288,
        "model": "custom-model",
    },
    "base_validation": {
        "expected_status_code": 200
    }
}

# Message templates
MESSAGES = {
    "system": {
        "role": "system",
        "content": "You are a helpful assistant that provides information about the UC Berkeley TAI project."
    },
    "system_cs61a": {
        "role": "system",
        "content": "You are a helpful assistant for the CS61A course at UC Berkeley."
    },
    "user_tai": {
        "role": "user",
        "content": "Tell me about the UC Berkeley TAI project."
    },
    "user_tai_about": {
        "role": "user",
        "content": "What is the UC Berkeley TAI project about?"
    },
    "user_cs61a": {
        "role": "user",
        "content": "What topics are covered in CS61A?"
    }
}

# Reference templates
REFERENCES = [
    {
        "id": "call_abc123",
        "type": "function",
        "function": {
            "name": "reference",
            "arguments": "{\"title\":\"UC Berkeley TAI Homepage\",\"url\":\"https://berkeley-tai.org\"}"
        }
    },
    {
        "id": "call_def456",
        "type": "function",
        "function": {
            "name": "reference",
            "arguments": "{\"title\":\"AI Safety Research at Berkeley\",\"url\":\"https://berkeley-tai.org/research\"}"
        }
    }
]

# Content templates
CONTENT = {
    "tai_description": "The UC Berkeley Trustworthy AI (TAI) project is a research initiative focused on ensuring that AI systems are safe, reliable, and aligned with human values. The project brings together researchers from computer science, statistics, and other fields to address challenges in AI safety and alignment.",
    "cs61a_description": "CS61A at UC Berkeley covers the fundamentals of computer programming, including abstraction, recursion, higher-order functions, and programming paradigms. The course primarily uses Python, and also introduces students to Scheme and SQL."
}


def ensure_dir(path: Path) -> None:
    """Ensure directory exists."""
    path.mkdir(parents=True, exist_ok=True)


def save_json(data: Dict, path: Path) -> None:
    """Save data as JSON file."""
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Created: {path}")


def create_meta(name: str, description: str, tags: List[str], **kwargs) -> Dict:
    """Create metadata section of a fixture."""
    meta = TEMPLATES["base_meta"].copy()
    meta["name"] = name
    meta["description"] = description
    meta["tags"] = tags
    meta.update(kwargs)
    return meta


def create_request_fixture(filename: str, meta: Dict, request_data: Dict) -> None:
    """Create a request fixture file."""
    dir_path = FIXTURES_DIR / "endpoints" / "completions" / "requests"
    ensure_dir(dir_path)

    fixture = {
        "meta": meta,
        "data": request_data,
    }

    if "validation" in meta:
        fixture["validation"] = meta.pop("validation")

    save_json(fixture, dir_path / filename)


def create_response_fixture(filename: str, meta: Dict, response_data: Union[Dict, List], validation: Optional[Dict] = None) -> None:
    """Create a response fixture file."""
    dir_path = FIXTURES_DIR / "endpoints" / "completions" / "responses"
    ensure_dir(dir_path)

    fixture = {
        "meta": meta,
        "data": response_data,
    }

    if validation:
        fixture["validation"] = validation

    save_json(fixture, dir_path / filename)


def create_rag_request(streaming: bool = False, course: str = "TAI") -> Dict:
    """Create RAG request data."""
    request = TEMPLATES["base_request"].copy()
    messages = []

    # System message based on course
    if not streaming:
        if course == "CS61A":
            messages.append(MESSAGES["system_cs61a"])
        else:
            messages.append(MESSAGES["system"])

    # User message based on course
    if course == "CS61A":
        messages.append(MESSAGES["user_cs61a"])
    elif streaming:
        messages.append(MESSAGES["user_tai_about"])
    else:
        messages.append(MESSAGES["user_tai"])

    request["messages"] = messages
    request["stream"] = streaming
    request["rag"] = True

    # Add course parameter if CS61A
    if course == "CS61A":
        request["course"] = "CS61A"

    return request


def create_non_rag_request(streaming: bool = False, course: str = "TAI") -> Dict:
    """Create non-RAG request data."""
    request = create_rag_request(streaming, course)
    request["rag"] = False
    return request


def create_non_streaming_response(with_references: bool = False, course: str = "TAI") -> Dict:
    """Create non-streaming response data."""
    response = TEMPLATES["base_response"].copy()
    response["object"] = "chat.completion"

    # Select content based on course
    content = CONTENT["cs61a_description"] if course == "CS61A" else CONTENT["tai_description"]

    choices = [{
        "index": 0,
        "message": {
            "role": "assistant",
            "content": content
        },
        "finish_reason": "stop"
    }]

    if with_references:
        choices[0]["message"]["tool_calls"] = REFERENCES

    response["choices"] = choices
    response["usage"] = {
        "prompt_tokens": 20,
        "completion_tokens": 100,
        "total_tokens": 120
    }

    if with_references:
        response["system_fingerprint"] = "fp_44709d6fcb"

    return response


def create_streaming_chunks(with_references: bool = False, course: str = "TAI") -> List[Dict]:
    """Create streaming response chunks."""
    base_chunk = {
        "id": "chatcmpl-123",
        "object": "chat.completion.chunk",
        "created": 1677652288,
        "model": "custom-model",
    }

    # Add system fingerprint for requests with references
    if with_references:
        base_chunk["system_fingerprint"] = "fp_44709d6fcb"

    # Select content based on course
    content = CONTENT["cs61a_description"] if course == "CS61A" else CONTENT["tai_description"]

    # Break content into chunks
    content_chunks = []

    # Simplistic chunking for example purposes
    words = content.split()
    chunk_size = 5
    for i in range(0, len(words), chunk_size):
        chunk = " ".join(words[i:i+chunk_size])
        content_chunks.append(chunk)

    # Create chunks list
    chunks = []

    # Role chunk
    role_chunk = base_chunk.copy()
    role_chunk["choices"] = [{
        "index": 0,
        "delta": {
            "role": "assistant"
        },
        "finish_reason": None
    }]
    chunks.append(role_chunk)

    # Content chunks
    for i, chunk in enumerate(content_chunks):
        # Insert reference after second chunk if requested
        if with_references and i == 2:
            ref_chunk = base_chunk.copy()
            ref_chunk["choices"] = [{
                "index": 0,
                "delta": {
                    "tool_calls": [REFERENCES[0]]
                },
                "finish_reason": None
            }]
            chunks.append(ref_chunk)

        content_chunk = base_chunk.copy()
        content_chunk["choices"] = [{
            "index": 0,
            "delta": {
                "content": chunk + " "
            },
            "finish_reason": None
        }]
        chunks.append(content_chunk)

        # Insert second reference toward the end if requested
        if with_references and i == len(content_chunks) - 2:
            ref_chunk = base_chunk.copy()
            ref_chunk["choices"] = [{
                "index": 0,
                "delta": {
                    "tool_calls": [REFERENCES[1]]
                },
                "finish_reason": None
            }]
            chunks.append(ref_chunk)

    # Stop chunk
    stop_chunk = base_chunk.copy()
    stop_chunk["choices"] = [{
        "index": 0,
        "delta": {},
        "finish_reason": "stop"
    }]
    chunks.append(stop_chunk)

    return chunks


def clean_fixtures():
    """Remove existing fixture files."""
    paths = [
        FIXTURES_DIR / "endpoints" / "completions" / "requests",
        FIXTURES_DIR / "endpoints" / "completions" / "responses"
    ]

    for path in paths:
        if path.exists():
            for file in path.glob("*.json"):
                file.unlink()
                print(f"Removed: {file}")


def generate_all_fixtures():
    """Generate all fixtures for the API."""
    # ------- Non-Streaming Requests -------

    # Standard RAG request (non-streaming)
    rag_request_meta = create_meta(
        "RAG Request",
        "A non-streaming completion request with RAG enabled using OpenAI format",
        ["request", "non-streaming", "rag", "openai-format"],
        validation={
            "expected_status_code": 200,
            "content_type": "application/json"
        }
    )
    create_request_fixture(
        "rag_request.json",
        rag_request_meta,
        create_rag_request(streaming=False)
    )

    # Standard non-RAG request
    non_rag_request_meta = create_meta(
        "Non-RAG Request",
        "A non-streaming completion request without RAG enabled",
        ["request", "non-streaming", "no-rag", "openai-format"],
        validation={
            "expected_status_code": 200,
            "content_type": "application/json"
        }
    )
    create_request_fixture(
        "non_rag_request.json",
        non_rag_request_meta,
        create_non_rag_request(streaming=False)
    )

    # CS61A non-streaming RAG request
    cs61a_request_meta = create_meta(
        "CS61A RAG Request",
        "A course-specific non-streaming request with RAG enabled",
        ["request", "non-streaming", "rag", "course-specific", "openai-format"],
        validation={
            "expected_status_code": 200,
            "content_type": "application/json"
        }
    )
    create_request_fixture(
        "cs61a_rag_request.json",
        cs61a_request_meta,
        create_rag_request(streaming=False, course="CS61A")
    )

    # ------- Streaming Requests -------

    # Streaming RAG request
    streaming_rag_request_meta = create_meta(
        "Streaming RAG Request",
        "A streaming completion request with RAG enabled using OpenAI format",
        ["request", "streaming", "rag", "openai-format"],
        validation={
            "expected_status_code": 200,
            "content_type": "text/event-stream"
        }
    )
    create_request_fixture(
        "streaming_rag_request.json",
        streaming_rag_request_meta,
        create_rag_request(streaming=True)
    )

    # Streaming non-RAG request
    streaming_non_rag_request_meta = create_meta(
        "Streaming Non-RAG Request",
        "A streaming completion request without RAG enabled",
        ["request", "streaming", "no-rag", "openai-format"],
        validation={
            "expected_status_code": 200,
            "content_type": "text/event-stream"
        }
    )
    create_request_fixture(
        "streaming_non_rag_request.json",
        streaming_non_rag_request_meta,
        create_non_rag_request(streaming=True)
    )

    # CS61A streaming RAG request
    cs61a_streaming_request_meta = create_meta(
        "CS61A Streaming RAG Request",
        "A course-specific streaming request with RAG enabled",
        ["request", "streaming", "rag", "course-specific", "openai-format"],
        validation={
            "expected_status_code": 200,
            "content_type": "text/event-stream"
        }
    )
    create_request_fixture(
        "cs61a_streaming_rag_request.json",
        cs61a_streaming_request_meta,
        create_rag_request(streaming=True, course="CS61A")
    )

    # ------- Non-Streaming Responses -------

    # Non-streaming RAG response
    non_streaming_rag_response_meta = create_meta(
        "Non-Streaming RAG Response",
        "A non-streaming response with references as tool calls",
        ["non-streaming", "response", "rag", "references", "tool-calls"]
    )
    non_streaming_validation = {
        "expected_status_code": 200,
        "expected_content": CONTENT["tai_description"],
        "expected_tool_call_count": 2
    }
    create_response_fixture(
        "non_streaming_rag_response.json",
        non_streaming_rag_response_meta,
        create_non_streaming_response(with_references=True),
        non_streaming_validation
    )

    # Non-streaming non-RAG response
    non_streaming_non_rag_response_meta = create_meta(
        "Non-Streaming Non-RAG Response",
        "A standard non-streaming response without references",
        ["non-streaming", "response", "no-rag"]
    )
    non_rag_validation = {
        "expected_status_code": 200,
        "expected_content": CONTENT["tai_description"],
        "expected_tool_call_count": 0
    }
    create_response_fixture(
        "non_streaming_non_rag_response.json",
        non_streaming_non_rag_response_meta,
        create_non_streaming_response(with_references=False),
        non_rag_validation
    )

    # CS61A non-streaming RAG response
    cs61a_non_streaming_response_meta = create_meta(
        "CS61A Non-Streaming RAG Response",
        "A course-specific non-streaming response with references",
        ["non-streaming", "response", "rag",
            "references", "tool-calls", "course-specific"]
    )
    cs61a_non_streaming_validation = {
        "expected_status_code": 200,
        "expected_content": CONTENT["cs61a_description"],
        "expected_tool_call_count": 2
    }
    create_response_fixture(
        "cs61a_non_streaming_rag_response.json",
        cs61a_non_streaming_response_meta,
        create_non_streaming_response(with_references=True, course="CS61A"),
        cs61a_non_streaming_validation
    )

    # ------- Streaming Responses -------

    # Streaming RAG response
    streaming_rag_response_meta = create_meta(
        "Streaming RAG Response",
        "A streaming completion response with references as tool calls integrated in the stream",
        ["streaming", "response", "rag", "references", "tool-calls"]
    )
    streaming_validation = {
        "expected_status_code": 200,
        "expected_content_chunks": len(CONTENT["tai_description"].split()) // 5 + 1,
        "expected_tool_call_chunks": 2,
        "expected_content": CONTENT["tai_description"]
    }
    create_response_fixture(
        "streaming_rag_response.json",
        streaming_rag_response_meta,
        create_streaming_chunks(with_references=True),
        streaming_validation
    )

    # Streaming non-RAG response
    streaming_non_rag_response_meta = create_meta(
        "Streaming Non-RAG Response",
        "A streaming completion response without references",
        ["streaming", "response", "no-rag"]
    )
    streaming_non_rag_validation = {
        "expected_status_code": 200,
        "expected_content_chunks": len(CONTENT["tai_description"].split()) // 5 + 1,
        "expected_tool_call_chunks": 0,
        "expected_content": CONTENT["tai_description"]
    }
    create_response_fixture(
        "streaming_non_rag_response.json",
        streaming_non_rag_response_meta,
        create_streaming_chunks(with_references=False),
        streaming_non_rag_validation
    )

    # CS61A streaming RAG response
    cs61a_streaming_response_meta = create_meta(
        "CS61A Streaming RAG Response",
        "A course-specific streaming response with references",
        ["streaming", "response", "rag", "references",
            "tool-calls", "course-specific"]
    )
    cs61a_streaming_validation = {
        "expected_status_code": 200,
        "expected_content_chunks": len(CONTENT["cs61a_description"].split()) // 5 + 1,
        "expected_tool_call_chunks": 2,
        "expected_content": CONTENT["cs61a_description"]
    }
    create_response_fixture(
        "cs61a_streaming_rag_response.json",
        cs61a_streaming_response_meta,
        create_streaming_chunks(with_references=True, course="CS61A"),
        cs61a_streaming_validation
    )


def main():
    parser = argparse.ArgumentParser(
        description="Generate test fixtures for the AI chatbot backend API")
    parser.add_argument("--clean", action="store_true",
                        help="Remove existing fixture files before generating new ones")
    args = parser.parse_args()

    if args.clean:
        clean_fixtures()

    generate_all_fixtures()
    print("Fixture generation complete!")


if __name__ == "__main__":
    main()
