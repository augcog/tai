# this file contain functions to convert input audio data to text using vllm whisper engine
import time
from ai_chatbot_backend.app.core.models.chat_completion import VoiceMessage
from faster_whisper import WhisperModel
import io
import soundfile as sf


def audio_to_text(
        audio_message: VoiceMessage,
        engine: WhisperModel,
        sample_rate: int = 24000
) -> str:
    """
    Convert audio message to text using Whisper model.
    """

    audio_buffer = io.BytesIO()
    sf.write(audio_buffer, audio_message.content, sample_rate, format='WAV')
    segments, _ = engine.transcribe(audio_buffer, beam_size=5)
    transcription = " ".join(segment.text for segment in segments)
    return transcription.strip()
