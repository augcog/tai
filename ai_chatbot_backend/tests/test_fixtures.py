"""
Test suite for validating fixtures.

This module tests that the fixtures conform to the expected formats and schemas.
"""

import pytest

# Import the load_fixtures function directly from the current directory
# We need to use a relative import that pytest can handle
from tests.conftest import load_fixtures


class TestFixtureStructure:
    """Test the structure of fixtures."""

    def test_request_fixtures_structure(self, fixtures):
        """Test that request fixtures have the correct structure."""
        for fixture in fixtures["requests"]:
            path = fixture.get("_path", "unknown")

            # For the new simplified structure, the fixture is the direct request payload
            # Check required fields for all requests
            assert "course" in fixture, f"Missing 'course' field in {path}"
            assert "messages" in fixture, f"Missing 'messages' field in {path}"
            assert "temperature" in fixture, f"Missing 'temperature' field in {path}"
            assert "stream" in fixture, f"Missing 'stream' field in {path}"

            # Check messages structure
            assert isinstance(fixture["messages"], list), (
                f"'messages' should be a list in {path}"
            )
            assert len(fixture["messages"]) > 0, (
                f"'messages' should not be empty in {path}"
            )

            for message in fixture["messages"]:
                assert "role" in message, f"Missing 'role' in message in {path}"
                assert "content" in message, f"Missing 'content' in message in {path}"
                assert message["role"] in [
                    "system",
                    "user",
                    "assistant",
                ], f"Invalid role '{message['role']}' in {path}"

            # Check rag field if present
            if "rag" in fixture:
                assert isinstance(fixture["rag"], bool), (
                    f"'rag' should be a boolean in {path}"
                )

    def test_response_fixtures_structure(self, fixtures):
        """Test that response fixtures have the correct structure."""
        for fixture in fixtures["responses"]:
            path = fixture.get("_path", "unknown")

            # Check required sections
            assert "meta" in fixture, f"Missing 'meta' section in {path}"
            assert "data" in fixture, f"Missing 'data' section in {path}"

            # Check meta fields
            meta = fixture["meta"]
            assert "name" in meta, f"Missing 'name' in meta section of {path}"
            assert "description" in meta, (
                f"Missing 'description' in meta section of {path}"
            )
            assert "tags" in meta, f"Missing 'tags' in meta section of {path}"
            assert isinstance(meta["tags"], list), f"'tags' should be a list in {path}"

            data = fixture["data"]

            # If it's streaming, data should be a list of chunks
            if "streaming" in meta.get("tags", []):
                assert isinstance(data, list), (
                    f"Data should be a list for streaming response in {path}"
                )
                for chunk in data:
                    assert "id" in chunk, f"Missing 'id' in chunk in {path}"
                    assert "object" in chunk, f"Missing 'object' in chunk in {path}"
                    assert "choices" in chunk, f"Missing 'choices' in chunk in {path}"

                    for choice in chunk["choices"]:
                        assert "index" in choice, f"Missing 'index' in choice in {path}"
                        assert "delta" in choice, f"Missing 'delta' in choice in {path}"
            else:
                # Non-streaming responses
                assert isinstance(data, dict), (
                    f"Data should be a dict for non-streaming response in {path}"
                )
                assert "id" in data, f"Missing 'id' in data in {path}"
                assert "object" in data, f"Missing 'object' in data in {path}"
                assert "choices" in data, f"Missing 'choices' in data in {path}"

                for choice in data["choices"]:
                    assert "index" in choice, f"Missing 'index' in choice in {path}"
                    assert "message" in choice, f"Missing 'message' in choice in {path}"
                    assert "finish_reason" in choice, (
                        f"Missing 'finish_reason' in choice in {path}"
                    )

            # Check for tool_calls or references if they are part of the tags
            if "references" in meta.get("tags", []) or "tool-calls" in meta.get(
                "tags", []
            ):
                if "streaming" in meta.get("tags", []):
                    # For streaming responses, find chunks with tool_calls
                    tool_calls_found = False
                    for chunk in data:
                        for choice in chunk["choices"]:
                            if "delta" in choice and "tool_calls" in choice["delta"]:
                                tool_calls_found = True
                                for tool_call in choice["delta"]["tool_calls"]:
                                    assert "type" in tool_call, (
                                        f"Missing 'type' in tool_call in {path}"
                                    )
                                    assert "function" in tool_call, (
                                        f"Missing 'function' in tool_call in {path}"
                                    )

                    assert tool_calls_found, (
                        f"No tool_calls found in streaming response tagged with 'references' in {path}"
                    )
                else:
                    # For non-streaming responses
                    for choice in data["choices"]:
                        assert "message" in choice, (
                            f"Missing 'message' in choice in {path}"
                        )
                        assert "tool_calls" in choice["message"], (
                            f"Missing 'tool_calls' in message in {path}"
                        )

                        for tool_call in choice["message"]["tool_calls"]:
                            assert "type" in tool_call, (
                                f"Missing 'type' in tool_call in {path}"
                            )
                            assert "function" in tool_call, (
                                f"Missing 'function' in tool_call in {path}"
                            )

    def test_validation_sections(self, fixtures):
        """Test that validation sections are correctly formatted."""
        # For responses, we still check validation sections
        for fixture in fixtures["responses"]:
            path = fixture.get("_path", "unknown")

            # Check required sections for responses
            assert "meta" in fixture, f"Missing 'meta' section in {path}"
            assert "data" in fixture, f"Missing 'data' section in {path}"

            if "validation" in fixture:
                validation = fixture["validation"]

                # Common validation fields
                if "expected_status_code" in validation:
                    assert isinstance(validation["expected_status_code"], int), (
                        f"'expected_status_code' should be an integer in {path}"
                    )

                if "content_type" in validation:
                    assert isinstance(validation["content_type"], str), (
                        f"'content_type' should be a string in {path}"
                    )

                # Response-specific validation
                if "streaming" in fixture.get("meta", {}).get("tags", []):
                    if "expected_content_chunks" in validation:
                        assert isinstance(validation["expected_content_chunks"], int), (
                            f"'expected_content_chunks' should be an integer in {path}"
                        )

                    if "expected_tool_call_chunks" in validation:
                        assert isinstance(
                            validation["expected_tool_call_chunks"], int
                        ), f"'expected_tool_call_chunks' should be an integer in {path}"


def test_fixture_loading():
    """Test that fixtures can be loaded."""
    fixtures = load_fixtures()
    assert "requests" in fixtures
    assert "responses" in fixtures
    assert len(fixtures["requests"]) > 0, "No request fixtures found"
    assert len(fixtures["responses"]) > 0, "No response fixtures found"

    # Return a proper assertion rather than the fixtures to avoid pytest warnings
    assert True


if __name__ == "__main__":
    pytest.main(["-v", __file__])
