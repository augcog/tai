import json
import unittest

from jsonschema import validate

from app.api.v1.utils.stream_processing import (
    openai_format_stream,
    extract_text_and_references_from_openai_format,
)
from tests.common.test_utils.openai_format_validation import (
    TOOL_CALL_SCHEMA,
    validate_stream_chunks,
    assert_contains_reference_markers,
)


class TestOpenAIFormat(unittest.TestCase):
    def test_openai_format_stream_with_references(self):
        """Test conversion of token stream to OpenAI format with reference markers"""

        # Simulate a generator with tokens containing a reference marker and final references
        def get_mock_data():
            return [
                json.dumps({"type": "token", "data": "Hello"}) + "\n",
                json.dumps({"type": "token", "data": " world"}) + "\n",
                json.dumps({"type": "token", "data": " [1]"})
                + "\n",  # Reference marker
                json.dumps({"type": "final", "references": ["https://example.com"]})
                + "\n",
            ]

        def mock_generator():
            for item in get_mock_data():
                yield item

        # Convert to OpenAI format - the function consumes the generator twice
        result = list(openai_format_stream(mock_generator()))

        # Parse the chunks
        chunks = [json.loads(chunk.strip()) for chunk in result]

        # Validate chunk structure against OpenAI schema
        validate_stream_chunks([chunk.strip() for chunk in result])

        # First chunk should have role
        self.assertEqual(chunks[0]["choices"][0]["delta"]["role"], "assistant")

        # Content chunks
        self.assertEqual(chunks[1]["choices"][0]["delta"]["content"], "Hello")
        self.assertEqual(chunks[2]["choices"][0]["delta"]["content"], " world")
        self.assertEqual(chunks[3]["choices"][0]["delta"]["content"], " [1]")

        # After reference marker, should have a tool call
        tool_call_chunk_found = False
        for chunk in chunks:
            delta = chunk["choices"][0]["delta"]
            if delta.get("tool_calls"):
                tool_call_chunk_found = True
                tool_call = delta["tool_calls"][0]
                # Validate tool call structure
                validate(instance=tool_call, schema=TOOL_CALL_SCHEMA)
                self.assertEqual(tool_call["type"], "function")
                self.assertEqual(tool_call["function"]["name"], "add_reference")
                args = json.loads(tool_call["function"]["arguments"])
                self.assertEqual(args["url"], "https://example.com")
                break

        self.assertTrue(
            tool_call_chunk_found, "Expected to find a chunk with tool calls"
        )

        # Last chunk should have finish_reason
        self.assertEqual(chunks[-1]["choices"][0]["finish_reason"], "stop")

        # Common OpenAI format fields should be present
        for chunk in chunks:
            self.assertEqual(chunk["object"], "chat.completion.chunk")
            self.assertTrue("id" in chunk)
            self.assertTrue("created" in chunk)
            self.assertTrue("model" in chunk)
            self.assertTrue("system_fingerprint" in chunk)

    def test_extract_references_from_openai_format(self):
        """Test extracting content and references from OpenAI format stream with tool calls"""

        # Simulate an OpenAI formatted stream with tool calls for references
        def mock_openai_stream():
            yield (
                json.dumps(
                    {
                        "id": "chatcmpl-123",
                        "object": "chat.completion.chunk",
                        "created": 1694268190,
                        "model": "custom-model",
                        "system_fingerprint": "fp_custom",
                        "choices": [
                            {
                                "index": 0,
                                "delta": {"role": "assistant"},
                                "finish_reason": None,
                            }
                        ],
                    }
                )
                + "\n"
            )

            yield (
                json.dumps(
                    {
                        "id": "chatcmpl-123",
                        "object": "chat.completion.chunk",
                        "created": 1694268190,
                        "model": "custom-model",
                        "system_fingerprint": "fp_custom",
                        "choices": [
                            {
                                "index": 0,
                                "delta": {"content": "Hello"},
                                "finish_reason": None,
                            }
                        ],
                    }
                )
                + "\n"
            )

            yield (
                json.dumps(
                    {
                        "id": "chatcmpl-123",
                        "object": "chat.completion.chunk",
                        "created": 1694268190,
                        "model": "custom-model",
                        "system_fingerprint": "fp_custom",
                        "choices": [
                            {
                                "index": 0,
                                "delta": {"content": " world [1]"},
                                "finish_reason": None,
                            }
                        ],
                    }
                )
                + "\n"
            )

            # Tool call for reference
            yield (
                json.dumps(
                    {
                        "id": "chatcmpl-123",
                        "object": "chat.completion.chunk",
                        "created": 1694268190,
                        "model": "custom-model",
                        "system_fingerprint": "fp_custom",
                        "choices": [
                            {
                                "index": 0,
                                "delta": {
                                    "tool_calls": [
                                        {
                                            "id": "call_abc123",
                                            "type": "function",
                                            "function": {
                                                "name": "add_reference",
                                                "arguments": '{"number":1,"title":"Reference 1","url":"https://example.com"}',
                                            },
                                        }
                                    ]
                                },
                                "finish_reason": None,
                            }
                        ],
                    }
                )
                + "\n"
            )

            yield (
                json.dumps(
                    {
                        "id": "chatcmpl-123",
                        "object": "chat.completion.chunk",
                        "created": 1694268190,
                        "model": "custom-model",
                        "system_fingerprint": "fp_custom",
                        "choices": [{"index": 0, "delta": {}, "finish_reason": "stop"}],
                    }
                )
                + "\n"
            )

        # Validate each chunk
        chunks = list(mock_openai_stream())
        parsed_chunks = [json.loads(chunk.strip()) for chunk in chunks]

        # Validate structure of each chunk
        validate_stream_chunks([chunk.strip() for chunk in chunks])

        # Validate tool call
        tool_call_chunk = parsed_chunks[3]  # Fourth chunk contains the tool call
        tool_call = tool_call_chunk["choices"][0]["delta"]["tool_calls"][0]
        validate(instance=tool_call, schema=TOOL_CALL_SCHEMA)

        # Extract text and references
        content, references = extract_text_and_references_from_openai_format(
            mock_openai_stream()
        )

        # Check content
        self.assertEqual(content, "Hello world [1]")

        # Check references
        self.assertEqual(len(references), 1)
        self.assertEqual(references[0], "https://example.com")

        # Check reference pattern in content
        assert_contains_reference_markers(content)


if __name__ == "__main__":
    unittest.main()
