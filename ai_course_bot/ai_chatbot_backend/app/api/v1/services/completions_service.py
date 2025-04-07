from ..schemas.file_completion import CompletionResponse


def create_completion(fileId: str, prompt: str, user: dict, rag: bool) -> CompletionResponse:
    """
    Create a chat completion for a file.
    Returns None if the file is not found (simulate error).
    """
    # For example purposes, assume only fileId "file123" exists.
    if fileId != "file123":
        return None
    return CompletionResponse(
        completionId="xyz789",
        content="Summary of the main points...",
    )
