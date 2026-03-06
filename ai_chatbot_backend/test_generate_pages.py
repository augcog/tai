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



SPEECH_SYSTEM_PROMPT = """\
You are TAI, a university tutor for {course_code}.
You are speaking aloud to a student, explaining a topic page by page.
Generate ONLY what you would say — natural, conversational, like a real lecture.

Rules:
- No markdown, no code fences, no bullet lists, no brackets.
- When referencing code, describe it in words (e.g., "a function called make_adder that takes n").
- Vary your pacing and transitions naturally.
- Use the teaching purpose as your guide for HOW to explain.
- Use the bullet points as your guide for WHAT to cover.
- Keep it focused — this is one page of a multi-page lesson."""


async def generate_speech_script(engine, label: str, course_code: str,
                                 user_content: str) -> str:
    """Stream a speech script from the model, printing tokens as they arrive."""
    system = SPEECH_SYSTEM_PROMPT.format(course_code=course_code)
    messages = [
        {"role": "system", "content": system},
        {"role": "user", "content": user_content},
    ]
    stream = await engine(
        user_content,
        messages=messages,
        stream=True,
        temperature=0.7,
        max_tokens=800,
    )
    print(f"\n--- SPEECH ({label}) ---")
    full_text = ""
    async for chunk in stream:
        if chunk.choices:
            token = chunk.choices[0].delta.content
            if token:
                print(token, end="", flush=True)
                full_text += token
    print("\n---")
    return full_text


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

    # State tracking
    outline_data = None       # Set when outline.complete arrives (may have empty bullets)
    outline_titles = []       # Page titles from outline["outline"] array
    page_purposes = {}        # {page_idx: purpose} from page.start events
    total_pages = 0
    current_page_idx = -1
    current_page_title = ""

    # Buffer page speech data until outline arrives (so Intro speech goes first)
    buffered_page_speeches = []  # [(page_idx, page_title, sub_bullets), ...]

    async def _generate_page_speech(page_idx, page_title, sub_bullets):
        """Generate a speech script for a content page (frontend page_idx+1)."""
        purpose = page_purposes.get(page_idx, "")
        bullet_text = "\n".join(
            f"- {sb.get('point', sb) if isinstance(sb, dict) else sb}"
            for sb in sub_bullets
        )
        prev_pages = ", ".join(
            f"\"{t}\"" for t in outline_titles[:page_idx]
        ) if page_idx > 0 else "(this is the first page)"
        await generate_speech_script(
            openai_engine, f"Page {page_idx + 1}", course_code,
            f"Page {page_idx + 1} of {total_pages}: \"{page_title}\"\n"
            f"Teaching approach: {purpose}\n"
            f"Key points to cover:\n{bullet_text}\n"
            f"Previous pages covered: {prev_pages}\n\n"
            f"Generate what you would say aloud to teach this page."
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
            outline_data = outline
            outline_titles = outline.get("outline", [])
            total_pages = len(outline_titles)
            is_single_page = not outline.get("needs_multiple_pages", True)

            if is_single_page:
                # Single-page: no outline display, no intro speech
                # Just replay any buffered page speeches directly
                for (buf_idx, buf_title, buf_bullets) in buffered_page_speeches:
                    await _generate_page_speech(buf_idx, buf_title, buf_bullets)
                buffered_page_speeches = []
            else:
                # Multi-page: show outline + intro speech as before
                print(f"\n--- OUTLINE (Page 0): {outline.get('title', 'Untitled')} ---")
                for i, title in enumerate(outline_titles):
                    print(f"  Page {i+1}: {title}")

                page_titles = "\n".join(f"- {t}" for t in outline_titles)
                await generate_speech_script(
                    openai_engine, "Intro", course_code,
                    f"Student asked: \"{question}\"\n"
                    f"Lesson: \"{outline.get('title', '')}\" ({total_pages} pages)\n"
                    f"Pages:\n{page_titles}\n\n"
                    f"Generate a brief spoken intro welcoming the student and previewing the lesson."
                )

                for (buf_idx, buf_title, buf_bullets) in buffered_page_speeches:
                    await _generate_page_speech(buf_idx, buf_title, buf_bullets)
                buffered_page_speeches = []

        elif evt_type == "page.start":
            current_page_idx = evt["page_index"]
            current_page_title = evt["point"]
            page_purposes[current_page_idx] = evt.get("purpose", "")
            print(f"\n{'='*60}")
            print(f"PAGE {current_page_idx + 1}: {current_page_title}")
            print(f"{'='*60}")

        elif evt_type == "page.bullets":
            sub_bullets = evt.get("sub_bullets", [])
            print(f"\n  Bullet points:")
            for j, sb in enumerate(sub_bullets):
                if isinstance(sb, dict):
                    print(f"    {j+1}. {sb.get('point', sb)}")
                else:
                    print(f"    {j+1}. {sb}")
            print()

            # Generate speech — buffer if outline not yet available
            if outline_data is None:
                buffered_page_speeches.append(
                    (current_page_idx, current_page_title, sub_bullets)
                )
            else:
                await _generate_page_speech(current_page_idx, current_page_title, sub_bullets)

        elif evt_type == "page.block_type":
            if evt.get("block_type") == "not_readable":
                print("\n  [Code/Formula block]")

        elif evt_type == "response.citation.open":
            cid = evt.get("citation_id", "?")
            quote = evt.get("quote_text")
            if quote:
                print(f"\n  [ref:{cid}] \"{quote}\"")
            else:
                print(f"\n  [ref:{cid}]")

        elif evt_type == "response.citation.close":
            pass

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
