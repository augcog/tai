import pytest

from app.api.v1.endpoints.completions import extract_text_and_references


@pytest.mark.parametrize(
    "endpoint, stream, expected_ref",
    [
        ("/v1/completions", False, ["https://github.com/augcog/tai"]),
        ("/v1/completions/no-rag", False, []),
        ("/v1/completions", True, ["https://github.com/augcog/tai"]),
        ("/v1/completions/no-rag", True, []),
    ],
)
def test_create_completion(client_unit, tai_trivia_question, endpoint, stream, expected_ref):
    payload = {
        "course": "CS61A",
        "messages": [{"role": "user", "content": tai_trivia_question}],
        "temperature": 0.7,
        "stream": stream,
    }
    response = client_unit.post(endpoint, json=payload)
    assert response.status_code == 200

    if stream:
        # For streaming responses, verify the content-type and extract streamed messages.
        assert response.headers["content-type"].startswith("application/json")
        content, references = extract_text_and_references(response.iter_lines())
        assert content, "Expected non-empty content"
        assert references is not None, "Expected references in the final message"
        assert isinstance(references, list), "Expected references to be a list in the final message"
    else:
        # For non-streaming responses, decode the JSON.
        data = response.json()
        assert "content" in data
        assert "references" in data
        content = data["content"]
        references = data["references"]
        assert isinstance(references, list), "Expected references to be a list in the final message"

    # Common assertions for both streaming and non-streaming responses.
    assert "uc berkeley" in content.lower()
    assert "tai" in content.lower()
    assert "teaching" in content.lower()

    # Additional assertions for provided extra keywords.
    for keyword in expected_ref:
        assert any(keyword in ref for ref in references), f"Expected keyword '{keyword}' in references"
