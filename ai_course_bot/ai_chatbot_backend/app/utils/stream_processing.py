import json
import re
from typing import Tuple, List, Any, Generator, Dict, Optional
from app.schemas.completion import ChatCompletionChunk, ToolCall

# Pattern to match reference numbers in text content
# Format: [1], [2], etc.
REFERENCE_PATTERN = re.compile(r'\[(\d+)\]')


def openai_format_stream(generator_response: Any) -> Generator[str, None, None]:
    """
    Process the generator of JSON strings from generate_chat_response into OpenAI-compatible format.

    Args:
        generator_response: A generator that yields JSON strings with token or final messages

    Yields:
        OpenAI-compatible chat completion chunk messages in JSON format
    """
    # First chunk with just the role
    first_chunk = ChatCompletionChunk.create_chunk(role="assistant")
    yield json.dumps(first_chunk.model_dump()) + "\n"

    # Store accumulated tokens for reference detection
    accumulated_text = ""
    seen_references = set()  # Track reference numbers we've already processed
    reference_urls = {}  # Map reference numbers to URLs

    # Collect all message objects to avoid consuming the generator twice
    messages = []
    for json_str in generator_response:
        try:
            messages.append(json.loads(json_str))
        except Exception as e:
            print(f"Error parsing chunk: {e}")

    # First pass: extract and map references
    for data in messages:
        if data.get("type") == "final":
            references = data.get("references", [])
            # Create a mapping of reference number to URL
            for i, ref in enumerate(references):
                if ref:  # Only include non-empty references
                    reference_urls[i+1] = ref

    # Second pass: process tokens and generate chunks
    for data in messages:
        if data.get("type") == "token":
            token = data.get("data", "")
            accumulated_text += token

            # Check if this token completes a reference marker
            reference_markers = REFERENCE_PATTERN.findall(accumulated_text)

            # If we found new reference markers and have not processed them yet
            new_tool_calls = []
            for ref_num_str in reference_markers:
                try:
                    ref_num = int(ref_num_str)
                    # Only process if we haven't seen this reference and it's in our mapping
                    if ref_num not in seen_references and ref_num in reference_urls:
                        seen_references.add(ref_num)
                        url = reference_urls[ref_num]
                        # Create a tool call for this reference
                        new_tool_calls.append(
                            ChatCompletionChunk.create_reference_tool_call(
                                f"Reference {ref_num}",
                                url,
                                ref_num  # Pass the reference number
                            )
                        )
                except ValueError:
                    continue

            # First yield the content token
            content_chunk = ChatCompletionChunk.create_chunk(content=token)
            yield json.dumps(content_chunk.model_dump()) + "\n"

            # Then, if we have new tool calls, yield them
            if new_tool_calls:
                tool_calls_chunk = ChatCompletionChunk.create_chunk(
                    tool_calls=new_tool_calls)
                yield json.dumps(tool_calls_chunk.model_dump()) + "\n"

        # No need to handle the "final" case here, as we already processed references

    # Final chunk with finish_reason
    final_chunk = ChatCompletionChunk.create_chunk(finish_reason="stop")
    yield json.dumps(final_chunk.model_dump()) + "\n"


def extract_text_and_references_from_openai_format(openai_stream: Generator[str, None, None]) -> Tuple[str, List[str]]:
    """
    Extract content text and reference URLs from an OpenAI-formatted stream.

    Args:
        openai_stream: A generator that yields OpenAI-formatted JSON strings

    Returns:
        Tuple containing the content text and list of reference URLs
    """
    content = ""
    references = []
    # Track reference number to URL mapping
    reference_map = {}

    for line in openai_stream:
        try:
            data = json.loads(line)
            # Extract content if present
            delta = data["choices"][0]["delta"]
            if "content" in delta and delta["content"] is not None:
                content += delta["content"]

            # Extract references from tool calls
            if "tool_calls" in delta and delta["tool_calls"]:
                for tool_call in delta["tool_calls"]:
                    if (tool_call["type"] == "function" and
                            tool_call["function"]["name"] == "add_reference"):
                        try:
                            args = json.loads(
                                tool_call["function"]["arguments"])
                            if args.get("url"):
                                url = args["url"]
                                # Get the reference number from arguments if available
                                if "number" in args:
                                    ref_num = args["number"]
                                    # Store in the mapping
                                    reference_map[ref_num] = url
                                references.append(url)
                        except json.JSONDecodeError:
                            continue
        except (json.JSONDecodeError, KeyError, IndexError):
            continue

    # Reorder references based on reference number if available
    if reference_map:
        # Use the reference_map to order references by number
        ordered_references = []
        for i in range(1, max(reference_map.keys()) + 1):
            if i in reference_map:
                ordered_references.append(reference_map[i])
        # If we have a complete ordered list, use it
        if len(ordered_references) == len(references):
            references = ordered_references

    return content, references
