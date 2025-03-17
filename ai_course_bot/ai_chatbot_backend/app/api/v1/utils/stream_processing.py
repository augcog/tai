import json
from typing import Tuple, List, Any


def extract_text_and_references(generator_response: Any) -> Tuple[str, List[str]]:
    """
    Process the generator of JSON strings from generate_chat_response.
    Aggregates token messages into full text and extracts references from the final message.
    
    Args:
        generator_response: A generator that yields JSON strings with token or final messages
        
    Returns:
        Tuple containing:
        - full_text: The aggregated text from all token messages
        - references: A list of references from the final message
    """
    tokens = []
    references = []
    for json_str in generator_response:
        try:
            data = json.loads(json_str)
        except Exception as e:
            print(f"Error parsing chunk: {e}")
            continue

        if data.get("type") == "token":
            tokens.append(data.get("data", ""))
        elif data.get("type") == "final":
            references = data.get("references", [])
    full_text = "".join(tokens)
    return full_text, references
