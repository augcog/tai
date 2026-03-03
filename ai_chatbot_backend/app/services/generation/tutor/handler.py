import json
import re
from typing import Optional, Set

from app.services.generation.base_handler import BaseStreamHandler, StreamContext, TransformResult
from app.services.generation.parser import BlockStreamEvent, extract_answers, extract_answers_with_citations


class TutorHandler(BaseStreamHandler):
    """
    Step 3 handler for tutor mode (TEXT_CHAT_TUTOR + VOICE_TUTOR).

    - Extracts markdown from JSON blocks via extract_answers()
    - JSON-based citation extraction with regex fallback
    - Audio interleaving controlled by audio_response flag (inherited from base)
    - Emits citation open/close events at block boundaries via extract_answers_with_citations()
    """

    def transform_delta(self, channel: str, full_text: str, ctx: StreamContext) -> Optional[str]:
        if channel != "final":
            # For analysis channel, return raw delta
            return full_text[len(ctx.previous_channels.get(channel, "")):]

        # For final channel, extract markdown from JSON blocks
        current_answer_text = extract_answers(full_text)
        delta = current_answer_text[len(ctx.previous_answer_text):]
        ctx.previous_answer_text = current_answer_text
        return delta if delta.strip() else None

    def transform_delta_with_citations(
        self, channel: str, full_text: str, ctx: StreamContext
    ) -> TransformResult:
        """Block-aware transform that produces interleaved citation + text events."""
        if channel != "final":
            text = full_text[len(ctx.previous_channels.get(channel, "")):]
            if text and text.strip():
                return TransformResult(events=[BlockStreamEvent(text_delta=text)])
            return TransformResult()

        # Use block-aware parser — returns events already in correct order
        return TransformResult(
            events=extract_answers_with_citations(full_text, ctx.block_stream_state)
        )

    def extract_references(self, final_text: str) -> Set[int]:
        """JSON-based citation extraction with regex fallback."""
        mentioned = set()
        try:
            text = final_text.strip()
            # Remove markdown code blocks if present
            if text.startswith('```'):
                text = re.sub(r'^```(?:json)?\s*\n?', '', text)
                text = re.sub(r'\n?```\s*$', '', text)

            json_data = json.loads(text)

            # Array structure: [{"reference": {"number": 1, ...}, ...}]
            if isinstance(json_data, list):
                for segment in json_data:
                    ref = segment.get('reference')
                    if ref is not None and isinstance(ref, dict) and 'number' in ref:
                        mentioned.add(int(ref['number']))

            # Blocks structure: {"blocks": [{"citations": [{"id": 1}]}]}
            elif isinstance(json_data, dict) and isinstance(json_data.get("blocks"), list):
                for block in json_data.get("blocks", []):
                    if not isinstance(block, dict):
                        continue
                    citations = block.get("citations", [])
                    if not isinstance(citations, list):
                        continue
                    for citation in citations:
                        if not isinstance(citation, dict):
                            continue
                        if "id" not in citation:
                            continue
                        try:
                            mentioned.add(int(citation["id"]))
                        except (TypeError, ValueError):
                            continue

            # Fallback for old structure
            elif isinstance(json_data, dict) and 'mentioned_contexts' in json_data:
                for context in json_data.get('mentioned_contexts', []):
                    if isinstance(context, dict) and 'reference' in context:
                        mentioned.add(int(context['reference']))

            print(f"\n[INFO] Mentioned references from JSON: {mentioned}")

        except (json.JSONDecodeError, ValueError, KeyError) as e:
            print(f"\n[WARNING] Failed to parse JSON output, falling back to regex: {e}")
            # Fall back to regex
            pattern = re.compile(
                r'(?:\[Reference:\s*([\d,\s]+)\]'
                r'|\breference\s+(\d+(?:(?:\s*,\s*|\s*(?:and|&)\s*)\d+)*))',
                re.IGNORECASE
            )
            mentioned = {
                int(n)
                for m in pattern.finditer(final_text)
                for n in re.findall(r'\d+', m.group(1) or m.group(2))
            }

        return mentioned


class OutlineHandler(BaseStreamHandler):
    """
    Step 3 handler for outline tutor mode.

    Streams raw JSON output (no block extraction or citation open/close events).
    After stream ends, extracts reference IDs from the outline bullets.
    """

    def transform_delta(self, channel: str, full_text: str, ctx: StreamContext) -> Optional[str]:
        # Pass through raw delta for all channels
        delta = full_text[len(ctx.previous_channels.get(channel, "")):]
        return delta if delta.strip() else None

    def transform_delta_with_citations(
        self, channel: str, full_text: str, ctx: StreamContext
    ) -> TransformResult:
        """No citation open/close events — just text deltas."""
        text = full_text[len(ctx.previous_channels.get(channel, "")):]
        if text and text.strip():
            return TransformResult(events=[BlockStreamEvent(text_delta=text)])
        return TransformResult()

    def extract_references(self, final_text: str) -> Set[int]:
        """Extract all reference IDs from outline bullets."""
        mentioned = set()
        try:
            text = final_text.strip()
            if text.startswith('```'):
                text = re.sub(r'^```(?:json)?\s*\n?', '', text)
                text = re.sub(r'\n?```\s*$', '', text)

            json_data = json.loads(text)

            # Outline structure: {"title": "...", "bullets": [{"references": [1, 2]}]}
            if isinstance(json_data, dict) and isinstance(json_data.get("bullets"), list):
                for bullet in json_data["bullets"]:
                    if not isinstance(bullet, dict):
                        continue
                    refs = bullet.get("references", [])
                    if isinstance(refs, list):
                        for ref_id in refs:
                            try:
                                mentioned.add(int(ref_id))
                            except (TypeError, ValueError):
                                continue

            print(f"\n[INFO] Outline mentioned references: {mentioned}")

        except (json.JSONDecodeError, ValueError, KeyError) as e:
            print(f"\n[WARNING] Failed to parse outline JSON: {e}")

        return mentioned
