# Test Fixtures

This directory contains fixtures used for testing the AI chatbot backend API. The fixtures are organized in a way that makes them useful for:

1. **Automated testing** - Used directly by test cases
2. **Documentation examples** - Referenced in docs to show API usage
3. **Postman collection generation** - Can be converted to Postman collections
4. **Frontend integration testing** - Example payloads for frontend tests

## Directory Structure

```
fixtures/
├── endpoints/              # Organized by API endpoint
│   ├── completions/        # /v1/completions endpoint
│   │   ├── requests/       # Request payloads
│   │   └── responses/      # Response payloads (with variants)
│   └── ...                 # Other endpoints
├── schemas/                # Schema examples
│   ├── request_schemas/    # Example objects matching request schemas
│   └── response_schemas/   # Example objects matching response schemas
└── scenarios/              # Complex multi-call scenarios
    ├── conversation/       # Complete conversations
    └── rag/                # RAG-specific scenarios
```

## Usage

### For Test Cases

```python
import json
from pathlib import Path

def load_fixture(path):
    fixture_path = Path(__file__).parent.parent / "fixtures" / path
    with open(fixture_path, "r") as f:
        return json.load(f)

# Example usage in a test
def test_completions_endpoint():
    request_data = load_fixture("endpoints/completions/requests/basic_rag.json")
    expected_response = load_fixture("endpoints/completions/responses/basic_rag.json")
    # Test code...
```

### For Postman Collection Generation

Run the collection generator script:

```bash
python scripts/generate_postman_collection.py
```

## Fixture Format

Each fixture follows a consistent format:

```json
{
  "meta": {
    "name": "Basic RAG request",
    "description": "Simple request with RAG enabled",
    "tags": ["rag", "streaming"]
  },
  "data": {
    // The actual payload
  }
}
```

The `meta` section is used for documentation and organization, while the `data` section contains the actual payload used in tests. 