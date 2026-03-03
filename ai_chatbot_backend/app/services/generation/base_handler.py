import base64
import io
import re
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import AsyncIterator, Optional, Set, Tuple, TYPE_CHECKING

import numpy as np

from app.core.models.chat_completion import (
    AudioSpec,
    AudioTranscript,
    CitationClose,
    CitationOpen,
    Done,
    Reference,
    ResponseDelta,
    ResponseReference,
    sse,
)
from app.services.generation.parser import BlockStreamState, extract_channels

if TYPE_CHECKING:
    from app.services.request_timer import RequestTimer

# Guard against streaming partial reference patterns
_PARTIAL_TAIL_GUARD = re.compile(r"""
(?ix)
(?:                                     # Match incomplete reference patterns at end
    \[\s*ref(?:erence)?\s*:?\s*         # [Reference: / [Ref / [reference
    (?:\d+(?:\s*(?:,|\band\b|&)\s*\d+)*)?   # Optional partial number sequence
    \s*(?:,|\band\b|&)?                 # Allow trailing separator
    \s*\Z
  |
    (?<![A-Za-z])(?:references?|ref)\s* # Prose style: reference / references / ref
    (?:\d+(?:\s*(?:,|\band\b|&)\s*\d+)*)?
    \s*(?:,|\band\b|&)?
    \s*\Z
  |
    \[\s*\Z                              # Just '[' at end
)
""", re.VERBOSE)


@dataclass
class TransformResult:
    """Return type for transform_delta_with_citations.
    Events are ordered: citation open/close and text deltas interleaved correctly.
    """
    events: list = field(default_factory=list)  # List[BlockStreamEvent]


@dataclass
class StreamContext:
    """Mutable state shared across pipeline stages for a single stream."""
    # Accumulated raw content
    accumulated_reasoning: str = ""
    accumulated_content: str = ""

    # Channel state (output of extract_channels)
    previous_channels: dict = field(default_factory=dict)

    # Content transformer state
    previous_answer_text: str = ""

    # Sequence counters
    text_seq: int = 0
    voice_seq: int = 0

    # Timing
    first_token_marked: bool = False

    # Audio state
    audio_messages: list = field(default_factory=list)
    previous_audio_index: int = -2

    # Block-level streaming state for citation open/close tracking
    block_stream_state: BlockStreamState = field(default_factory=BlockStreamState)


class BaseStreamHandler(ABC):
    """
    Shared streaming handler base class.

    Pipeline stages (per chunk):
    1. Accumulate raw chunks (reasoning_content + content)
    2. extract_channels() -> channels dict
    3. Compute deltas per channel
    4. PARTIAL_TAIL_GUARD check
    5. transform_delta() for mode-specific processing
    6. yield sse(ResponseDelta(...))
    7. Audio interleaving if audio_response is True

    Post-stream:
    8. extract_references() -> yield sse(ResponseReference(...))
    9. Timer report -> yield sse(Done()) + [DONE]
    """

    def __init__(
        self,
        stream: AsyncIterator,
        reference_list: list,
        audio_response: bool,
        course_code: Optional[str],
        audio_text: Optional[str],
        timer: Optional["RequestTimer"],
    ):
        self.stream = stream
        self.reference_list = reference_list
        self.audio_response = audio_response
        self.course_code = course_code
        self.audio_text = audio_text
        self.timer = timer
        self.ctx = StreamContext()

    @abstractmethod
    def transform_delta(self, channel: str, full_text: str, ctx: StreamContext) -> Optional[str]:
        """Given full accumulated channel text, return new delta to emit."""
        ...

    @abstractmethod
    def extract_references(self, final_text: str) -> Set[int]:
        """After stream ends, extract mentioned reference numbers."""
        ...

    def transform_delta_with_citations(
        self, channel: str, full_text: str, ctx: StreamContext
    ) -> TransformResult:
        """
        Extended transform that also produces citation lifecycle events.
        Default implementation delegates to transform_delta() with no citation events.
        Subclasses (TutorHandler) override this to produce citation events.
        """
        from app.services.generation.parser import BlockStreamEvent
        text = self.transform_delta(channel, full_text, ctx)
        if text and text.strip():
            return TransformResult(events=[BlockStreamEvent(text_delta=text)])
        return TransformResult()

    def get_audio_text(self, final_text: str, previous_index: int) -> Optional[Tuple[str, int]]:
        """Default sentence-boundary splitting for TTS. Shared by all voice modes."""
        last_sentence_end = final_text.rfind('. ')
        if last_sentence_end > previous_index + 2:
            audio_text = final_text[previous_index + 2:last_sentence_end + 2]
            if audio_text.strip():
                return audio_text, last_sentence_end
        return None

    async def run(self) -> AsyncIterator[str]:
        """Main pipeline generator. Yields SSE-formatted strings."""
        # Emit audio transcript if present
        if self.audio_text:
            yield sse(AudioTranscript(text=self.audio_text))

        # Per-chunk streaming loop
        async for output in self.stream:
            # Stage 1: Accumulate raw chunks
            if hasattr(output, 'choices') and output.choices:
                delta = output.choices[0].delta

                if hasattr(delta, 'reasoning_content') and delta.reasoning_content:
                    self.ctx.accumulated_reasoning += delta.reasoning_content

                if hasattr(delta, 'content') and delta.content:
                    self.ctx.accumulated_content += delta.content

            # Build full text for extract_channels
            if self.ctx.accumulated_reasoning:
                text = f"<think>{self.ctx.accumulated_reasoning}</think>{self.ctx.accumulated_content}"
            else:
                text = self.ctx.accumulated_content

            # Stage 2: Extract channels
            channels = extract_channels(text)
            if not channels:
                continue

            # Stage 3: Compute deltas
            chunks = {
                c: channels[c][len(self.ctx.previous_channels.get(c, "")):]
                for c in channels
                if channels[c] != self.ctx.previous_channels.get(c, "")
            }
            if not chunks:
                continue

            # Stage 4: Process each channel
            continue_flag = False
            for channel in chunks:
                chunk = chunks[channel]
                if not chunk.strip():
                    continue

                # PARTIAL_TAIL_GUARD check on final channel
                if channel == "final" and _PARTIAL_TAIL_GUARD.search(channels[channel][-100:]):
                    continue_flag = True
                    break

                # Stage 5: Strategy-specific content transformation (with citation events)
                result = self.transform_delta_with_citations(channel, channels[channel], self.ctx)

                # Stage 6: Emit events in order (citation open/close interleaved with text)
                for evt in result.events:
                    if evt.citation_close is not None:
                        yield sse(CitationClose(citation_id=evt.citation_close))
                    if evt.citation_open is not None:
                        yield sse(CitationOpen(
                            citation_id=evt.citation_open.citation_id,
                            quote_text=evt.citation_open.quote_text,
                        ))
                    if evt.text_delta is not None and evt.text_delta.strip():
                        if self.timer and not self.ctx.first_token_marked:
                            self.timer.mark("first_token")
                            self.ctx.first_token_marked = True
                        yield sse(ResponseDelta(
                            seq=self.ctx.text_seq, text_channel=channel, text=evt.text_delta
                        ))
                        self.ctx.text_seq += 1

            if continue_flag:
                continue

            self.ctx.previous_channels = channels

            # Stage 7: Audio interleaving
            if self.audio_response and 'final' in channels:
                async for audio_event in self._interleave_audio(channels['final']):
                    yield audio_event
        else:
            # for/else: flush remaining chunks after stream ends
            channels = self.ctx.previous_channels
            chunks = {
                c: channels[c][len(self.ctx.previous_channels.get(c, "")):]
                for c in channels
                if channels[c] != self.ctx.previous_channels.get(c, "")
            }
            for channel in chunks:
                chunk = chunks[channel]
                if not chunk.strip():
                    continue
                yield sse(ResponseDelta(seq=self.ctx.text_seq, text_channel=channel, text=chunk))
                self.ctx.text_seq += 1
                print(chunk, end="")

            # Flush remaining audio
            if self.audio_response and 'final' in channels:
                async for audio_event in self._flush_remaining_audio(channels['final']):
                    yield audio_event

        # Debug: print complete JSON output for tutor modes
        channels = self.ctx.previous_channels
        if self.__class__.__name__.endswith('TutorHandler') and 'final' in channels:
            import json as _json
            print("[DEBUG] Complete Original JSON Output:")
            try:
                print(_json.dumps(_json.loads(channels['final']), indent=2, ensure_ascii=False))
            except (ValueError, TypeError):
                print(channels['final'])

        # Close any still-open citation at end of stream
        if self.ctx.block_stream_state.active_citation_id is not None:
            yield sse(CitationClose(
                citation_id=self.ctx.block_stream_state.active_citation_id
            ))
            self.ctx.block_stream_state.active_citation_id = None

        # Stage 8: Extract and emit references
        mentioned_references = set()
        if 'final' in channels:
            mentioned_references = self.extract_references(channels['final'])

        references = []
        max_idx = len(self.reference_list)
        for i in sorted(mentioned_references):
            if 1 <= i <= max_idx:
                info_path, url, file_path, file_uuid, chunk_index = self.reference_list[i - 1]
                references.append(Reference(
                    reference_idx=i,
                    info_path=info_path,
                    url=url,
                    file_path=file_path,
                    file_uuid=file_uuid,
                    chunk_index=chunk_index
                ))
        if references:
            yield sse(ResponseReference(references=references))

        # Stage 9: Timer report + Done
        if self.timer:
            self.timer.mark("stream_complete")
            print(self.timer.report())

        yield sse(Done())
        yield "data: [DONE]\n\n"

    async def _interleave_audio(self, final_text: str) -> AsyncIterator[str]:
        """Interleave TTS audio chunks during streaming."""
        from app.services.audio.tts import audio_generator, get_speaker_name, convert_audio_to_base64

        result = self.get_audio_text(final_text, self.ctx.previous_audio_index)
        if result is None:
            return

        audio_text, new_index = result
        self.ctx.previous_audio_index = new_index

        messages_to_send = audio_text.split('. ')
        for msg in messages_to_send:
            if not msg.strip():
                continue
            self.ctx.audio_messages.append({"role": "user", "content": msg + '. '})
            print(f"\n[INFO] Audio text:")
            print(msg + '. ')

            speaker_name = get_speaker_name(self.course_code)
            audio_iterator = audio_generator(
                self.ctx.audio_messages, stream=True, speaker_name=speaker_name
            )
            audio_bytes_io = io.BytesIO()
            async for data in audio_iterator:
                yield sse(ResponseDelta(
                    seq=self.ctx.voice_seq, audio_b64=data, audio_spec=AudioSpec()
                ))
                self.ctx.voice_seq += 1
                audio_bytes = base64.b64decode(data)
                audio_bytes_io.write(audio_bytes)

            audio_data = np.frombuffer(audio_bytes_io.getvalue(), dtype=np.int16)
            audio2_base64 = convert_audio_to_base64(audio_data, 24000, target_format="wav")
            self.ctx.audio_messages.append({
                "role": "assistant",
                "content": [
                    {
                        "type": "input_audio",
                        "input_audio": {
                            "data": audio2_base64,
                            "format": "wav",
                        },
                    }
                ],
            })

    async def _flush_remaining_audio(self, final_text: str) -> AsyncIterator[str]:
        """Flush any remaining audio text after the stream ends."""
        from app.services.audio.tts import audio_generator, get_speaker_name, convert_audio_to_base64

        remaining = final_text[self.ctx.previous_audio_index + 2:]
        if not remaining.strip():
            return

        messages_to_send = remaining.split('. ')
        for msg in messages_to_send:
            if not msg.strip():
                continue
            self.ctx.audio_messages.append({"role": "user", "content": msg + '. '})
            print(f"\n[INFO] Audio text:")
            print(msg + '. ')

            speaker_name = get_speaker_name(self.course_code)
            audio_iterator = audio_generator(
                self.ctx.audio_messages, stream=True, speaker_name=speaker_name
            )
            audio_bytes_io = io.BytesIO()
            async for data in audio_iterator:
                yield sse(ResponseDelta(
                    seq=self.ctx.voice_seq, audio_b64=data, audio_spec=AudioSpec(),
                    speaker_name=speaker_name
                ))
                self.ctx.voice_seq += 1
                audio_bytes = base64.b64decode(data)
                audio_bytes_io.write(audio_bytes)

            audio_data = np.frombuffer(audio_bytes_io.getvalue(), dtype=np.int16)
            audio2_base64 = convert_audio_to_base64(audio_data, 24000, target_format="wav")
            self.ctx.audio_messages.append({
                "role": "assistant",
                "content": [
                    {
                        "type": "input_audio",
                        "input_audio": {
                            "data": audio2_base64,
                            "format": "wav",
                        },
                    }
                ],
            })
