"""
Combined pages pipeline: generate query -> OpenAI outline -> OpenAI page content.

Outline: OpenAI generates structured outline with needs_multiple_pages, outline titles, and bullets.
Page content: OpenAI generates block-based narration with sub_bullets, citations, and reference open/close.

Pipelined: page 0 content generation starts as soon as bullet 0 is parsed
from the streaming outline, rather than waiting for the entire outline.
"""
import asyncio
import json
import re
from typing import Any, AsyncIterator, List, Optional

from app.core.models.chat_completion import (
    CitationOpen,
    CitationClose,
    Done,
    GeneratePagesParams,
    OutlineComplete,
    PageBlockType,
    PageBullets,
    PageContentParams,
    PageDelta,
    PageError,
    PageReference,
    PageStart,
    Reference,
    ResponseReference,
    sse,
)
from app.services.generation.parser import (
    BlockStreamState,
    extract_answers_with_citations,
)


async def run_generate_pages_pipeline(
    params: GeneratePagesParams,
    openai_engine: Any,
    local_engine: Any,
) -> AsyncIterator[str]:
    """
    Combined pipeline: outline generation (OpenAI) -> per-page content (OpenAI).

    For each page, OpenAI generates block-based JSON with:
      - sub_bullets: knowledge sub-points for the page
      - blocks: narration with integrated citation open/close
    """
    from app.services.generation.tutor.query import build_tutor_context
    from app.services.generation.tutor.generate import call_tutor_model
    from app.services.generation.tutor.page_content.query import build_page_content_context
    from app.services.generation.tutor.page_content.generate import call_page_content_openai

    # === Step 1: Build context (RAG + query reformulation) ===
    context = await build_tutor_context(
        messages=params.messages,
        user_focus=None,
        answer_content=None,
        problem_content=None,
        course=params.course_code,
        engine=openai_engine,
        sid=params.sid,
        timer=None,
        audio_response=False,
    )

    # === Step 2: Emit all retrieved references upfront ===
    references = _build_references_from_list(context.reference_list)
    if references:
        yield sse(ResponseReference(references=references))

    # === Step 3: Start outline stream ===
    raw_stream = await call_tutor_model(
        context.messages, openai_engine, stream=True,
        audio_response=False, course=params.course_code,
        outline_mode=True,
    )

    # === Step 4: Background task — collect outline, push bullets to queue ===
    bullet_queue: asyncio.Queue = asyncio.Queue()
    outline_holder: dict = {}  # mutable container for the parsed outline

    async def _collect_outline():
        outline_text = ""
        parsed_count = 0
        titles_extracted = False
        async for chunk in raw_stream:
            if not hasattr(chunk, "choices") or not chunk.choices:
                continue
            delta = chunk.choices[0].delta
            content = getattr(delta, "content", None)
            if not content:
                continue
            outline_text += content

            # Detect outline titles early (before bullets finish)
            if not titles_extracted:
                titles = _extract_outline_titles(outline_text)
                if titles:
                    outline_holder["data"] = titles
                    titles_extracted = True

            # Try to extract newly completed bullets
            new_bullets = _extract_new_bullets(outline_text, parsed_count)
            for b in new_bullets:
                await bullet_queue.put(b)
                parsed_count += 1

        # Store the full parsed outline (overwrites early titles with complete data)
        parsed = _parse_outline(outline_text)
        if parsed:
            outline_holder["data"] = parsed
        elif not titles_extracted:
            print(f"[ERROR] Failed to parse outline JSON: {outline_text[:500]}")
            outline_holder["error"] = True

        # Sentinel: no more bullets
        await bullet_queue.put(None)

    outline_task = asyncio.create_task(_collect_outline())

    # === Step 5: Generate pages as bullets arrive from the queue ===
    page_idx = 0
    outline_emitted = False

    while True:
        bullet = await bullet_queue.get()
        if bullet is None:
            break

        try:
            page_refs = _resolve_page_references(
                bullet.get("references", []),
                context.reference_list,
            )

            page_params = PageContentParams(
                point=bullet["point"],
                purpose=bullet["purpose"],
                references=page_refs,
                course_code=params.course_code,
            )

            yield sse(PageStart(page_index=page_idx, point=bullet["point"], purpose=bullet["purpose"]))

            # --- Generate page content via OpenAI (block-based JSON, streaming) ---
            page_messages = build_page_content_context(page_params)
            page_stream = await call_page_content_openai(
                page_messages, openai_engine, course=params.course_code
            )

            page_text = ""
            sub_bullets_emitted = False
            block_state = BlockStreamState()
            seq = 0
            has_content = False

            async for chunk in page_stream:
                # Check if outline finished during page generation
                if not outline_emitted and outline_holder.get("data"):
                    yield sse(OutlineComplete(outline=outline_holder["data"]))
                    outline_emitted = True

                if not chunk.choices:
                    continue
                delta = chunk.choices[0].delta
                content = getattr(delta, "content", None)
                if not content:
                    continue

                page_text += content

                # Try to extract sub_bullets early (before blocks arrive)
                if not sub_bullets_emitted:
                    sub_bullets = _extract_sub_bullets(page_text)
                    if sub_bullets is not None:
                        yield sse(PageBullets(page_index=page_idx, sub_bullets=sub_bullets))
                        sub_bullets_emitted = True

                # Incrementally parse blocks for citation events + text deltas
                events = extract_answers_with_citations(page_text, block_state)
                for evt in events:
                    if evt.block_type is not None:
                        yield sse(PageBlockType(page_index=page_idx, block_type=evt.block_type))
                    if evt.citation_open is not None:
                        yield sse(CitationOpen(
                            citation_id=evt.citation_open.citation_id,
                            quote_text=evt.citation_open.quote_text or None,
                        ))
                    if evt.text_delta is not None:
                        yield sse(PageDelta(page_index=page_idx, seq=seq, text=evt.text_delta))
                        seq += 1
                        has_content = True
                    if evt.citation_close is not None:
                        yield sse(CitationClose(citation_id=evt.citation_close))

            # Final parse after stream completes — flush remaining events
            if page_text:
                events = extract_answers_with_citations(page_text, block_state)
                for evt in events:
                    if evt.block_type is not None:
                        yield sse(PageBlockType(page_index=page_idx, block_type=evt.block_type))
                    if evt.citation_open is not None:
                        yield sse(CitationOpen(
                            citation_id=evt.citation_open.citation_id,
                            quote_text=evt.citation_open.quote_text or None,
                        ))
                    if evt.text_delta is not None:
                        yield sse(PageDelta(page_index=page_idx, seq=seq, text=evt.text_delta))
                        seq += 1
                        has_content = True
                    if evt.citation_close is not None:
                        yield sse(CitationClose(citation_id=evt.citation_close))

            # Emit sub_bullets if not yet emitted (fallback: parse complete JSON)
            if not sub_bullets_emitted and page_text:
                sub_bullets = _extract_sub_bullets(page_text)
                if sub_bullets is not None:
                    yield sse(PageBullets(page_index=page_idx, sub_bullets=sub_bullets))

            # Check outline again after page generation
            if not outline_emitted and outline_holder.get("data"):
                yield sse(OutlineComplete(outline=outline_holder["data"]))
                outline_emitted = True

            if not has_content:
                print(f"[WARNING] Page {page_idx} produced no content tokens")
                yield sse(PageError(page_index=page_idx, error="Model produced no content for this page"))

        except Exception as e:
            print(f"[ERROR] Page {page_idx} generation failed: {e}")
            yield sse(PageError(page_index=page_idx, error=str(e)))

        page_idx += 1

    # === Step 6: Ensure outline.complete is emitted ===
    if not outline_emitted:
        if not outline_task.done():
            await outline_task
        if outline_holder.get("data"):
            yield sse(OutlineComplete(outline=outline_holder["data"]))
        elif outline_holder.get("error"):
            yield sse(PageError(page_index=-1, error="Failed to parse outline JSON"))

    yield sse(Done())
    yield "data: [DONE]\n\n"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _extract_outline_titles(text: str) -> Optional[dict]:
    """
    Extract title + outline array from partial outline JSON (before bullets finish).

    The outline JSON streams: needs_multiple_pages → title → outline[] → bullets[].
    Once the outline array is complete, we have all page titles needed for
    outline.complete and Intro speech, without waiting for bullet details.
    """
    # Try to extract the "outline" array (string items only)
    match = re.search(r'"outline"\s*:\s*(\[.*?\])', text, re.DOTALL)
    if not match:
        return None
    try:
        outline_arr = json.loads(match.group(1))
        if not isinstance(outline_arr, list) or not outline_arr:
            return None
    except json.JSONDecodeError:
        return None

    # Extract title
    title_match = re.search(r'"title"\s*:\s*"((?:[^"\\]|\\.)*)"', text)
    title = title_match.group(1) if title_match else ""

    # Extract needs_multiple_pages
    nmp_match = re.search(r'"needs_multiple_pages"\s*:\s*(true|false)', text)
    needs_multiple = nmp_match.group(1) == "true" if nmp_match else len(outline_arr) > 1

    return {
        "needs_multiple_pages": needs_multiple,
        "title": title,
        "outline": outline_arr,
        "bullets": [],  # bullets not yet available
    }


def _extract_sub_bullets(text: str) -> Optional[list]:
    """
    Extract the sub_bullets array from partial or complete page content JSON.

    Returns the list of sub-bullet strings if found and complete, else None.
    sub_bullets appears before blocks in the JSON, so it's available early in the stream.
    """
    # Try complete JSON parse first
    try:
        data = json.loads(text)
        if isinstance(data, dict) and "sub_bullets" in data:
            return data["sub_bullets"]
    except json.JSONDecodeError:
        pass

    # Streaming: try to extract the sub_bullets array from partial JSON
    match = re.search(r'"sub_bullets"\s*:\s*(\[.*?\])', text, re.DOTALL)
    if match:
        try:
            arr = json.loads(match.group(1))
            if isinstance(arr, list):
                return arr
        except json.JSONDecodeError:
            pass

    return None


def _extract_new_bullets(text: str, already_parsed: int) -> List[dict]:
    """
    Incrementally extract complete bullet objects from partial outline JSON.

    Tracks brace depth (with string/escape awareness) to detect complete
    {...} objects inside the "bullets" array.
    """
    # Find the start of the bullets array
    match = re.search(r'"bullets"\s*:\s*\[', text)
    if not match:
        return []

    start = match.end()
    bullets: List[dict] = []
    depth = 0
    obj_start = None
    in_string = False
    escape = False

    for i in range(start, len(text)):
        ch = text[i]

        if escape:
            escape = False
            continue
        if ch == '\\' and in_string:
            escape = True
            continue
        if ch == '"':
            in_string = not in_string
            continue
        if in_string:
            continue

        if ch == '{':
            if depth == 0:
                obj_start = i
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0 and obj_start is not None:
                try:
                    obj = json.loads(text[obj_start:i + 1])
                    if "point" in obj and "purpose" in obj:
                        bullets.append(obj)
                except json.JSONDecodeError:
                    pass
                obj_start = None

    # Return only newly parsed bullets
    return bullets[already_parsed:]


def _parse_outline(text: str) -> Optional[dict]:
    """Parse outline JSON from model output, handling markdown code fences."""
    text = text.strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*\n?", "", text)
        text = re.sub(r"\n?```\s*$", "", text)
    try:
        data = json.loads(text)
        if isinstance(data, dict) and "bullets" in data:
            return data
    except json.JSONDecodeError:
        pass
    return None


def _build_references_from_list(reference_list: list) -> List[Reference]:
    """Build Reference objects from all retrieved references."""
    references = []
    for i, ref_tuple in enumerate(reference_list, 1):
        info_path, url, file_path, file_uuid, chunk_index = ref_tuple
        references.append(Reference(
            reference_idx=i,
            info_path=info_path,
            url=url,
            file_path=file_path,
            file_uuid=file_uuid,
            chunk_index=chunk_index,
        ))
    return references


def _resolve_page_references(ref_ids: list, reference_list: list) -> List[PageReference]:
    """Convert outline reference integer IDs to PageReference objects for page content."""
    page_refs = []
    max_idx = len(reference_list)
    for ref_id in ref_ids:
        try:
            idx = int(ref_id)
        except (TypeError, ValueError):
            continue
        if 1 <= idx <= max_idx:
            _, _, _, file_uuid, chunk_index = reference_list[idx - 1]
            page_refs.append(PageReference(
                file_uuid=file_uuid,
                chunk_index=chunk_index,
            ))
    return page_refs
