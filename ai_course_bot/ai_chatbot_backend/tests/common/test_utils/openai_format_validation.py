import json
import re
from typing import List, Dict, Any, Union

from jsonschema import validate

# OpenAI chunk schema for validation
OPENAI_CHUNK_SCHEMA = {
    "type": "object",
    "required": ["id", "object", "created", "model", "system_fingerprint", "choices"],
    "properties": {
        "id": {"type": "string"},
        "object": {"type": "string", "enum": ["chat.completion.chunk"]},
        "created": {"type": "integer"},
        "model": {"type": "string"},
        "system_fingerprint": {"type": "string"},
        "choices": {
            "type": "array",
            "items": {
                "type": "object",
                "required": ["index", "delta"],
                "properties": {
                    "index": {"type": "integer"},
                    "delta": {"type": "object"},
                    "finish_reason": {"type": ["string", "null"]}
                }
            }
        }
    }
}

# Tool call schema
TOOL_CALL_SCHEMA = {
    "type": "object",
    "required": ["id", "type", "function"],
    "properties": {
        "id": {"type": "string"},
        "type": {"type": "string", "enum": ["function"]},
        "function": {
            "type": "object",
            "required": ["name", "arguments"],
            "properties": {
                "name": {"type": "string", "enum": ["add_reference"]},
                "arguments": {"type": "string"}
            }
        }
    }
}


def parse_chunk(chunk: Union[str, bytes]) -> Dict[str, Any]:
    """Parse a chunk into a JSON object, handling bytes or string.
    
    Args:
        chunk: The chunk data, either as string or bytes
        
    Returns:
        Parsed JSON object
    """
    if isinstance(chunk, bytes):
        return json.loads(chunk.decode('utf-8'))
    return json.loads(chunk)


def validate_stream_chunks(chunks: List[Union[str, bytes]]) -> None:
    """Validate a list of stream chunks against the OpenAI schema.
    
    Args:
        chunks: List of chunks (string or bytes)
        
    Raises:
        AssertionError: If validation fails
    """
    # Validate chunk structure against OpenAI schema
    for chunk in chunks:
        parsed = parse_chunk(chunk)
        validate(instance=parsed, schema=OPENAI_CHUNK_SCHEMA)


def validate_sample_chunks(chunks: List[Union[str, bytes]], sample_size: int = 3) -> None:
    """Validate a sample of chunks to ensure they conform to the OpenAI format.
    
    Args:
        chunks: List of chunks (string or bytes)
        sample_size: Number of content chunks to validate
        
    Raises:
        AssertionError: If validation fails
    """
    # Ensure we have at least one chunk, role chunk, and finish_reason chunk
    assert len(chunks) >= 3, "Expected at least 3 chunks in stream"

    # Parse first chunk (should have role)
    first_chunk = parse_chunk(chunks[0])
    validate(instance=first_chunk, schema=OPENAI_CHUNK_SCHEMA)
    assert "role" in first_chunk["choices"][0]["delta"], "First chunk should contain role"

    # Parse last chunk (should have finish_reason)
    last_chunk = parse_chunk(chunks[-1])
    validate(instance=last_chunk, schema=OPENAI_CHUNK_SCHEMA)
    assert last_chunk["choices"][0]["finish_reason"] == "stop", "Last chunk should have finish_reason 'stop'"

    # Validate a few content chunks
    for i in range(1, min(sample_size + 1, len(chunks) - 1)):
        chunk = parse_chunk(chunks[i])
        validate(instance=chunk, schema=OPENAI_CHUNK_SCHEMA)


def assert_contains_reference_markers(content: str) -> None:
    """Check if the content contains reference markers [n].
    
    Args:
        content: The text content to check
        
    Raises:
        AssertionError: If no reference markers are found
    """
    reference_pattern = re.compile(r'\[\d+\]')
    assert reference_pattern.search(content), "Expected reference markers [n] in content with RAG enabled"


def assert_has_tool_calls_for_references(chunks: List[Union[str, bytes]]) -> None:
    """Check if the stream contains tool calls for references.
    
    Args:
        chunks: List of chunks (string or bytes)
        
    Raises:
        AssertionError: If no tool calls are found or they have invalid structure
    """
    # Look for chunks with tool_calls
    tool_call_found = False
    for chunk in chunks:
        parsed = parse_chunk(chunk)
        delta = parsed["choices"][0]["delta"]
        if delta.get("tool_calls"):
            tool_call_found = True
            # Validate tool call structure
            tool_call = delta["tool_calls"][0]
            validate(instance=tool_call, schema=TOOL_CALL_SCHEMA)
            assert tool_call["function"]["name"] == "add_reference"
            # Validate arguments contains URL
            args = json.loads(tool_call["function"]["arguments"])
            assert "url" in args, "Tool call arguments should contain a URL"
            # If there's a 'number' field, it should be an integer
            if "number" in args:
                assert isinstance(args["number"], int), "Reference number should be an integer"
            break
    
    assert tool_call_found, "Expected to find tool calls for references with RAG enabled"


def assert_tool_calls_in_response(data: Dict[str, Any]) -> None:
    """Check if non-streaming response contains tool calls for references.
    
    Args:
        data: The response data
        
    Raises:
        AssertionError: If no tool calls are found or they have invalid structure
    """
    delta = data["choices"][0]["delta"]
    assert "tool_calls" in delta, "Expected tool_calls in response with RAG enabled"
    assert len(delta["tool_calls"]) > 0, "Expected at least one tool call"
    
    # Validate the tool call
    tool_call = delta["tool_calls"][0]
    validate(instance=tool_call, schema=TOOL_CALL_SCHEMA)
    
    # Validate arguments
    args = json.loads(tool_call["function"]["arguments"])
    assert "url" in args, "Tool call arguments should contain a URL"
    # If there's a 'number' field, it should be an integer
    if "number" in args:
        assert isinstance(args["number"], int), "Reference number should be an integer"


def extract_references_from_tool_calls(data: Dict[str, Any]) -> List[str]:
    """Extract reference URLs from tool calls in a response.
    
    Args:
        data: The response data
        
    Returns:
        List of reference URLs
    """
    references = []
    reference_map = {}  # Map reference numbers to URLs
    
    delta = data.get("choices", [{}])[0].get("delta", {})
    
    if delta.get("tool_calls"):
        for tool_call in delta.get("tool_calls"):
            if (tool_call.get("type") == "function" and 
                tool_call.get("function", {}).get("name") == "add_reference"):
                try:
                    args = json.loads(tool_call["function"]["arguments"])
                    if args.get("url"):
                        url = args["url"]
                        # If number is available, store in the mapping
                        if "number" in args:
                            reference_map[args["number"]] = url
                        references.append(url)
                except Exception:
                    continue
    
    # Reorder references based on reference number if available
    if reference_map:
        ordered_references = []
        for i in range(1, max(reference_map.keys()) + 1):
            if i in reference_map:
                ordered_references.append(reference_map[i])
        # If we have a complete ordered list, use it
        if len(ordered_references) == len(references):
            references = ordered_references
    
    return references


# Export validate for use in consumer modules
__all__ = [
    'OPENAI_CHUNK_SCHEMA',
    'TOOL_CALL_SCHEMA',
    'validate',
    'parse_chunk',
    'validate_stream_chunks',
    'validate_sample_chunks',
    'assert_contains_reference_markers',
    'assert_has_tool_calls_for_references',
    'assert_tool_calls_in_response',
    'extract_references_from_tool_calls'
]
