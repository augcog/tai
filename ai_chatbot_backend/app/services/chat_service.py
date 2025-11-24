from app.core.models.chat_completion import *
from typing import AsyncIterator, List, Any, Dict
import re
import os
import base64
import io
import soundfile as sf
import numpy as np
from openai import OpenAI


async def chat_stream_parser(
        stream: AsyncIterator,
        reference_list: List[str],
        audio: bool = False,
        messages: List[Message] = None,
        audio_text: str = None,
        engine: Any = None,
        old_sid: str = "",
        course_code: str = None,
        debug: bool = False
) -> AsyncIterator[str]:
    """
    Parse the streaming response from the vLLM chat model with reasoning support.

    Handles vLLM's OpenAI-compatible streaming format where:
    - delta.reasoning_content: Contains reasoning/thinking content (analysis channel)
    - delta.content: Contains final response content (final channel)

    The vLLM server with --reasoning-parser qwen3 flag separates these automatically.
    """
    if audio_text:
        yield sse(AudioTranscript(text=audio_text))

    # Accumulated content for each channel
    accumulated_analysis = ""
    accumulated_final = ""

    # Tracking for delta calculation and SSE sequencing
    text_seq = 0
    voice_seq = 0
    previous_index = -2
    audio_messages = []

    # Guard against streaming partial reference patterns
    PARTIAL_TAIL_GUARD = re.compile(r"""
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

    async for chunk in stream:
        if not chunk.choices:
            continue

        delta = chunk.choices[0].delta

        # Extract reasoning_content and content using getattr for safety
        # vLLM may use 'reasoning_content' or 'reasoning' depending on version
        reasoning = getattr(delta, "reasoning_content", None) or getattr(delta, "reasoning", None)
        content = getattr(delta, "content", None)

        # Process reasoning content (analysis channel)
        if reasoning:
            accumulated_analysis += reasoning
            # Check for partial reference patterns before yielding
            if not PARTIAL_TAIL_GUARD.search(accumulated_analysis[-100:]):
                yield sse(ResponseDelta(seq=text_seq, text_channel="analysis", text=reasoning))
                text_seq += 1
                if debug:
                    print(f"[analysis] {reasoning}", end="")

        # Process final content
        if content:
            accumulated_final += content
            # Check for partial reference patterns before yielding
            if not PARTIAL_TAIL_GUARD.search(accumulated_final[-100:]):
                yield sse(ResponseDelta(seq=text_seq, text_channel="final", text=content))
                text_seq += 1
                if debug:
                    print(f"[final] {content}", end="")

                # Handle audio TTS for final channel if enabled
                if audio:
                    last_sentence_end = accumulated_final.rfind('. ')
                    if last_sentence_end > previous_index + 2:
                        audio_text_chunk = accumulated_final[previous_index + 2:last_sentence_end + 2]
                        previous_index = last_sentence_end

                        if audio_text_chunk.strip():
                            messages_to_send = audio_text_chunk.split('. ')
                            for msg in messages_to_send:
                                if msg.strip():
                                    audio_messages.append({"role": "user", "content": msg + '. '})
                                    print(f"\n[INFO] Audio text: {msg}. ")
                                    speaker_name = get_speaker_name(course_code)
                                    audio_iterator = audio_generator(audio_messages, stream=True, speaker_name=speaker_name)
                                    audio_bytes_io = io.BytesIO()
                                    async for data in audio_iterator:
                                        yield sse(ResponseDelta(seq=voice_seq, audio_b64=data, audio_spec=AudioSpec()))
                                        voice_seq += 1
                                        audio_bytes = base64.b64decode(data)
                                        audio_bytes_io.write(audio_bytes)
                                    audio_data = np.frombuffer(audio_bytes_io.getvalue(), dtype=np.int16)
                                    audio2_base64 = convert_audio_to_base64(audio_data, 24000, target_format="wav")
                                    audio_messages.append({
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

    # Handle any remaining audio at end of stream
    if audio and accumulated_final:
        remaining_audio = accumulated_final[previous_index + 2:]
        if remaining_audio.strip():
            messages_to_send = remaining_audio.split('. ')
            for msg in messages_to_send:
                if msg.strip():
                    audio_messages.append({"role": "user", "content": msg + '. '})
                    print(f"\n[INFO] Audio text: {msg}. ")
                    speaker_name = get_speaker_name(course_code)
                    audio_iterator = audio_generator(audio_messages, stream=True, speaker_name=speaker_name)
                    audio_bytes_io = io.BytesIO()
                    async for data in audio_iterator:
                        yield sse(ResponseDelta(seq=voice_seq, audio_b64=data, audio_spec=AudioSpec(), speaker_name=speaker_name))
                        voice_seq += 1
                        audio_bytes = base64.b64decode(data)
                        audio_bytes_io.write(audio_bytes)
                    audio_data = np.frombuffer(audio_bytes_io.getvalue(), dtype=np.int16)
                    audio2_base64 = convert_audio_to_base64(audio_data, 24000, target_format="wav")
                    audio_messages.append({
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

    # Extract and yield references from the final content
    pattern = re.compile(
        r'(?:\[Reference:\s*([\d,\s]+)\]'
        r'|\breference\s+(\d+(?:(?:\s*,\s*|\s*(?:and|&)\s*)\d+)*))',
        re.IGNORECASE
    )

    mentioned_references = {
        int(n)
        for m in pattern.finditer(accumulated_final)
        for n in re.findall(r'\d+', m.group(1) or m.group(2))
    }
    print(f"\n[INFO] Mentioned references: {mentioned_references}")

    references = []
    max_idx = len(reference_list)
    for i in sorted(mentioned_references):
        if 1 <= i <= max_idx:
            info_path, url, file_path, file_uuid, chunk_index = reference_list[i - 1]
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

    yield sse(Done())
    yield "data: [DONE]\n\n"  # Final done message for SSE clients


def encode_base64_content_from_file(file_path: str) -> str:
    """Encode a content from a local file to base64 format."""
    with open(file_path, "rb") as audio_file:
        audio_base64 = base64.b64encode(audio_file.read()).decode("utf-8")
    return audio_base64


def convert_audio_to_base64(audio: np.ndarray,
                            sampling_rate: int,
                            target_format: str = "wav") -> str:
    audio_buffer = io.BytesIO()
    sf.write(audio_buffer, audio, sampling_rate, format=target_format)
    return base64.b64encode(audio_buffer.getvalue()).decode('utf-8')


def format_audio_text_message(audio_text: str) -> List[Dict]:
    """
    Format the audio text message into the expected structure for the chat model.
    """
    audio_text = re.sub(r'\[.*?\]', '', audio_text).replace('\n', ' ').strip()
    print(f"[INFO] Audio text after cleaning: {audio_text}")
    return [{"role": "user", "content": audio_text}]


def get_speaker_name(course_code: str) -> str:
    if course_code == "CS 61A":
        return "Professor John DeNero"
    elif course_code in ["ROAR Academy", "CS 294-137"]:
        return "Professor Allen Yang"
    else:
        return "Professor Allen Yang"  # Default speaker name


async def audio_generator(messages: List[Dict], stream: bool = True, speaker_name: str = None
) -> AsyncIterator[str]:
    """
    Parse the streaming response from the audio model and yield deltas.
    """
    data_dir = '/home/tai25/bot/tai/ai_chatbot_backend/voice_prompts'

    # Select voice prompt based on course_code
    if speaker_name == "Professor John DeNero":
        audio_file = "trees_54.wav"
        text_file = "trees_54.txt"
    elif speaker_name == "Professor Allen Yang":
        audio_file = "Allen_yang_voice.wav"
        text_file = "Allen_yang_voice.txt"
    else:
        audio_file = "Allen_yang_voice.wav"
        text_file = "Allen_yang_voice.txt"

    audio_path = os.path.join(data_dir, audio_file)
    audio_text_path = os.path.join(data_dir, text_file)
    with open(audio_text_path, "r") as f:
        audio_text = f.read()
    audio_base64 = encode_base64_content_from_file(audio_path)
    messages_add_to_begining = [
        {"role": "user", "content": audio_text},
        {
            "role": "assistant",
            "content": [
                {
                    "type": "input_audio",
                    "input_audio": {
                        "data": audio_base64,
                        "format": "wav",
                    },
                }
            ],
        }
    ]
    messages = messages_add_to_begining + messages
    if len(messages) > 3:
        messages = messages[:2] + messages[-1:]
    client = OpenAI(base_url='http://128.32.43.216:8000/v1', api_key='EMPTY')
    models = client.models.list()
    model = models.data[0].id
    chat_completion = client.chat.completions.create(
        messages=messages,
        model=model,
        max_completion_tokens=500,
        stream=stream,
        modalities=["text", "audio"],
        temperature=1.0,
        top_p=0.95,
        extra_body={"top_k": 50},
        stop=["<|eot_id|>", "<|end_of_text|>", "<|audio_eos|>"],
    )
    for chunk in chat_completion:
        if chunk.choices and hasattr(chunk.choices[0].delta, "audio") and chunk.choices[0].delta.audio:
            yield chunk.choices[0].delta.audio["data"]


async def tts_parsor(
        stream: AsyncIterator
) -> AsyncIterator[str]:
    """
    Parse the streaming response from the TTS model and yield deltas.
    """
    seq = 0
    async for data in stream:
        yield sse(ResponseDelta(seq=seq, audio_b64=data, audio_spec=AudioSpec()))
        seq += 1

    yield sse(Done())
    yield "data: [DONE]\n\n"  # Final done message for SSE clients
