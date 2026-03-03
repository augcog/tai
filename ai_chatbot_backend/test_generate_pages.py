"""
Quick test script for the generate-pages pipeline.
Run from the ai_chatbot_backend directory:

    python test_generate_pages.py "YOUR_COURSE_CODE" "Your question here"

Example:
    python test_generate_pages.py CS170 "Explain binary search trees"
"""
import asyncio
import json
import sys

from app.dependencies.model import initialize_model_engine, get_engine_for_mode
from app.core.models.chat_completion import GeneratePagesParams, Message
from app.services.generation.tutor.generate_pages import run_generate_pages_pipeline


async def main(course_code: str, question: str):
    print(f"\n{'='*60}")
    print(f"Course: {course_code}")
    print(f"Question: {question}")
    print(f"{'='*60}\n")

    # Get engines
    openai_engine = get_engine_for_mode("openai")
    local_engine = get_engine_for_mode("local")

    params = GeneratePagesParams(
        course_code=course_code,
        messages=[Message(role="user", content=question)],
        stream=True,
    )

    # Run pipeline and print each SSE event
    async for event_str in run_generate_pages_pipeline(params, openai_engine, local_engine):
        # Parse the SSE data line
        if not event_str.startswith("data: "):
            continue
        raw = event_str[len("data: "):].strip()
        if raw == "[DONE]":
            print("\n[DONE]")
            break

        evt = json.loads(raw)
        evt_type = evt.get("type", "")

        if evt_type == "response.reference":
            print(f"\n--- REFERENCES ({len(evt['references'])} found) ---")
            for ref in evt["references"]:
                print(f"  [{ref['reference_idx']}] {ref.get('file_path', 'N/A')}")

        elif evt_type == "outline.complete":
            outline = evt["outline"]
            print(f"\n--- OUTLINE: {outline.get('title', 'Untitled')} ---")
            for i, b in enumerate(outline.get("bullets", [])):
                print(f"  Page {i}: {b['point']}")
                print(f"          purpose: {b['purpose']}...")
                print(f"          refs: {b.get('references', [])}")

        elif evt_type == "page.start":
            print(f"\n{'='*60}")
            print(f"PAGE {evt['page_index']}: {evt['point']}")
            print(f"{'='*60}")

        elif evt_type == "page.delta":
            print(evt["text"], end="", flush=True)

        elif evt_type == "page.error":
            print(f"\n[ERROR] Page {evt['page_index']}: {evt['error']}")

        elif evt_type == "done":
            print("\n--- ALL DONE ---")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python test_generate_pages.py <course_code> <question>")
        print('Example: python test_generate_pages.py CS170 "Explain binary search trees"')
        sys.exit(1)

    course_code = sys.argv[1]
    question = " ".join(sys.argv[2:])
    asyncio.run(main(course_code, question))
