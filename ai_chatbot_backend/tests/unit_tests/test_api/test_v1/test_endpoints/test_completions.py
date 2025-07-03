import json
import re

import pytest

from app.api.v1.utils.stream_processing import (
    extract_text_and_references_from_openai_format,
)
from tests.common.test_utils.openai_format_validation import (
    validate,
    validate_sample_chunks,
    assert_contains_reference_markers,
    assert_has_tool_calls_for_references,
    assert_tool_calls_in_response,
    extract_references_from_tool_calls,
    OPENAI_CHUNK_SCHEMA,
    TOOL_CALL_SCHEMA,
    parse_chunk,
)


@pytest.mark.parametrize(
    "endpoint, stream, expected_ref, rag",
    [
        ("/v1/completions", False, ["https://github.com/augcog/tai"], True),
        ("/v1/completions", False, [], False),
        ("/v1/completions", True, ["https://github.com/augcog/tai"], True),
        ("/v1/completions", True, [], False),
    ],
)
def test_create_completion(
    client_unit, tai_trivia_question, endpoint, stream, expected_ref, rag
):
    """
    Test the completions endpoint with OpenAI format.
    Tests both streaming and non-streaming responses, with and without RAG.
    """
    payload = {
        "course": "CS61A",
        "messages": [{"role": "user", "content": tai_trivia_question}],
        "temperature": 0.7,
        "stream": stream,
        "rag": rag,
    }
    response = client_unit.post(endpoint, json=payload)
    assert response.status_code == 200

    if stream:
        # For streaming responses, verify the content-type and extract streamed messages.
        assert response.headers["content-type"].startswith("application/json")
        content, references = extract_text_and_references_from_openai_format(
            response.iter_lines()
        )
        assert content, "Expected non-empty content"
        assert isinstance(references, list), "Expected references to be a list"

        # Get all chunks for detailed validation
        chunks = list(response.iter_lines())
        print(f"\nNumber of chunks: {len(chunks)}")

        # Validate structure of a sample of chunks
        validate_sample_chunks(chunks)

        # Validate reference markers in content if RAG is enabled
        if rag:
            assert_contains_reference_markers(content)
            assert_has_tool_calls_for_references(chunks)
    else:
        # For non-streaming responses, decode the JSON.
        data = response.json()
        assert "choices" in data
        assert len(data["choices"]) > 0

        # Check for OpenAI format fields
        assert "id" in data
        assert "object" in data
        assert "created" in data
        assert "model" in data

        # Extract content and references from tool calls
        delta = data["choices"][0]["delta"]
        content = delta.get("content", "")
        references = extract_references_from_tool_calls(data)

        # Validate structure of the response
        if rag:
            assert_tool_calls_in_response(data)

    # Common assertions for both streaming and non-streaming responses.
    assert "uc berkeley" in content.lower()
    assert "tai" in content.lower()
    assert "teaching" in content.lower()

    if rag:
        # Additional assertions for provided extra keywords.
        for keyword in expected_ref:
            assert any(keyword in ref for ref in references), (
                f"Expected keyword '{keyword}' in references"
            )
    else:
        assert not references, "Expected no references"


def validate_sample_chunks(chunks, sample_size=3):
    """Validate a sample of chunks to ensure they conform to the OpenAI format."""
    # Ensure we have at least one chunk, role chunk, and finish_reason chunk
    assert len(chunks) >= 3, "Expected at least 3 chunks in stream"

    # Parse first chunk (should have role)
    first_chunk = parse_chunk(chunks[0])
    validate(instance=first_chunk, schema=OPENAI_CHUNK_SCHEMA)
    assert "role" in first_chunk["choices"][0]["delta"], (
        "First chunk should contain role"
    )

    # Parse last chunk (should have finish_reason)
    last_chunk = parse_chunk(chunks[-1])
    validate(instance=last_chunk, schema=OPENAI_CHUNK_SCHEMA)
    assert last_chunk["choices"][0]["finish_reason"] == "stop", (
        "Last chunk should have finish_reason 'stop'"
    )

    # Validate a few content chunks
    for i in range(1, min(sample_size + 1, len(chunks) - 1)):
        chunk = parse_chunk(chunks[i])
        validate(instance=chunk, schema=OPENAI_CHUNK_SCHEMA)


def parse_chunk(chunk):
    """Parse a chunk into a JSON object, handling bytes or string."""
    if isinstance(chunk, bytes):
        return json.loads(chunk.decode("utf-8"))
    return json.loads(chunk)


def assert_contains_reference_markers(content):
    """Check if the content contains reference markers [n]."""
    reference_pattern = re.compile(r"\[\d+\]")
    assert reference_pattern.search(content), (
        "Expected reference markers [n] in content with RAG enabled"
    )


def assert_has_tool_calls_for_references(chunks):
    """Check if the stream contains tool calls for references."""
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
            break

    assert tool_call_found, (
        "Expected to find tool calls for references with RAG enabled"
    )


def assert_tool_calls_in_response(data):
    """Check if non-streaming response contains tool calls for references."""
    delta = data["choices"][0]["delta"]
    assert "tool_calls" in delta, "Expected tool_calls in response with RAG enabled"
    assert len(delta["tool_calls"]) > 0, "Expected at least one tool call"

    # Validate the tool call
    tool_call = delta["tool_calls"][0]
    validate(instance=tool_call, schema=TOOL_CALL_SCHEMA)

    # Validate arguments
    args = json.loads(tool_call["function"]["arguments"])
    assert "url" in args, "Tool call arguments should contain a URL"
