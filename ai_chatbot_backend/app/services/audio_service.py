# This file contains functions to convert input audio data to text using vLLM Whisper server
from openai import OpenAI, AsyncOpenAI
import io
import json
import soundfile as sf
from app.core.models.chat_completion import VoiceMessage, sse, AudioTranscript, Done
from app.config import settings
from typing import Union, AsyncIterator
import numpy as np


def audio_to_text(
        audio_message: VoiceMessage,
        client: OpenAI,
        stream: bool = False,
        sample_rate: int = 24000
) -> str:
    """
    Convert audio message to text using vLLM Whisper server via OpenAI API (synchronous).

    Note: vLLM's Whisper implementation currently has a 30-second audio limit.
    """

    audio_buffer = io.BytesIO()
    # Convert List[float] directly to numpy array (no base64 decoding needed)
    audio_array = np.array(audio_message.content, dtype=np.float32)
    sf.write(audio_buffer, audio_array, sample_rate, format='WAV')
    # CRITICAL: Seek back to beginning after writing
    audio_buffer.seek(0)
    # Set a filename for the buffer (required by OpenAI API)
    audio_buffer.name = "audio.wav"

    # Use OpenAI transcription API (non-streaming)
    transcription = client.audio.transcriptions.create(
        model=settings.vllm_whisper_model,
        file=audio_buffer,
        response_format="text"
    )

    # Return the full transcription text
    if hasattr(transcription, 'text'):
        return transcription.text.strip()
    # Handle case where vLLM returns JSON string instead of plain text
    result = str(transcription).strip()
    try:
        parsed = json.loads(result)
        if isinstance(parsed, dict) and 'text' in parsed:
            return parsed['text'].strip()
    except (json.JSONDecodeError, TypeError):
        pass
    return result


async def audio_to_text_streaming(
        audio_message: VoiceMessage,
        sample_rate: int = 24000
) -> AsyncIterator[str]:
    """
    Convert audio message to text using vLLM Whisper server with streaming support.

    Uses AsyncOpenAI client for true streaming transcription.
    """
    # Create async client
    async_client = AsyncOpenAI(
        base_url=settings.vllm_whisper_url,
        api_key=settings.vllm_api_key
    )

    audio_buffer = io.BytesIO()
    audio_array = np.array(audio_message.content, dtype=np.float32)
    sf.write(audio_buffer, audio_array, sample_rate, format='WAV')
    audio_buffer.seek(0)
    audio_buffer.name = "audio.wav"

    # Use streaming transcription
    transcription_stream = await async_client.audio.transcriptions.create(
        model=settings.vllm_whisper_model,
        file=audio_buffer,
        stream=True
    )

    # Stream the transcription chunks
    async for chunk in transcription_stream:
        if chunk.choices:
            delta = chunk.choices[0].get("delta", {})
            content = delta.get("content")
            if content:
                yield content


async def audio_stream_parser(audio_message: VoiceMessage, sample_rate: int = 24000) -> AsyncIterator[str]:
    """
    Parse audio into streaming transcription format for SSE response.
    """
    accumulated_text = ""
    async for chunk in audio_to_text_streaming(audio_message, sample_rate):
        accumulated_text += chunk
        yield sse(AudioTranscript(text=chunk))

    yield sse(Done())
    yield "data: [DONE]\n\n"
