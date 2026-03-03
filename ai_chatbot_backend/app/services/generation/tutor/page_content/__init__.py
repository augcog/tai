from typing import Any, AsyncIterator

from app.core.models.chat_completion import Done, PageContentParams, ResponseDelta, sse


async def run_page_content_pipeline(
    params: PageContentParams,
    engine: Any,
) -> AsyncIterator[str]:
    """
    Page-content pipeline: fetch chunks -> build prompt -> call model -> stream markdown.

    Simpler than the tutor pipeline — no RAG retrieval, no JSON schema,
    no citation lifecycle. Just plain markdown streaming via SSE.
    """
    from .query import build_page_content_context
    from .generate import call_page_content_model

    # Step 1: Build messages from references + prompt
    messages = build_page_content_context(params)

    # Step 2: Call model and stream SSE
    seq = 0
    async for chunk in call_page_content_model(messages, engine):
        if not chunk.choices:
            continue
        delta = chunk.choices[0].delta

        # Skip reasoning/thinking content — only stream final content
        content = getattr(delta, "content", None)
        if content:
            yield sse(ResponseDelta(seq=seq, text=content))
            seq += 1

    yield sse(Done())
    yield "data: [DONE]\n\n"
