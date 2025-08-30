# this file contain functions to convert input audio data to text using vllm whisper engine
from faster_whisper import WhisperModel
from faster_whisper.transcribe import Segment
import io
import soundfile as sf
from app.core.models.chat_completion import *
from typing import Union, AsyncIterator
import base64
import numpy as np


def audio_to_text(
        audio_message: VoiceMessage,
        engine: WhisperModel,
        stream: bool = False,
        sample_rate: int = 24000
) -> Union[str, Segment]:
    """
    Convert audio message to text using Whisper model.
    """

    audio_buffer = io.BytesIO()
    audio_bytes = base64.b64decode(audio_message.content)
    audio_array=np.array(audio_bytes, dtype=np.float16)
    sf.write(audio_buffer, audio_array, sample_rate, format='WAV')
    segments, _ = engine.transcribe(audio_buffer, beam_size=5)
    if stream:
        return segments
    else:
        transcription = " ".join(segment.text for segment in segments)
        return transcription.strip()


async def audio_stream_parser(segments) -> AsyncIterator[str]:
    """
    Parse audio segments into a streaming format.
    """
    for segment in segments:
        yield sse(AudioTranscript(text=segment.text))
    yield sse(Done())
    yield "data: [DONE]\n\n"
