# Standard python libraries
import re
import os
import base64
import io
from typing import List, Dict, AsyncIterator
from pathlib import Path

# Third-party libraries
import soundfile as sf
import numpy as np
from openai import OpenAI

# Local libraries
from app.core.models.chat_completion import sse, ResponseDelta, AudioSpec, Done
from app.config import settings


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
    # remove all the [] in text
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
    Uses vLLM TTS server configured via VLLM_TTS_URL environment variable.
    """
    # Dynamic path based on current file location
    _current_file = Path(__file__).resolve()
    _backend_root = _current_file.parent.parent.parent.parent  # Navigate up to ai_chatbot_backend/
    data_dir = str(_backend_root / 'voice_prompts')

    # Select voice prompt based on course_code
    if speaker_name == "Professor John DeNero":
        audio_file = "trees_54_original.wav"
        text_file = "trees_54.txt"
    elif speaker_name == "Professor Allen Yang":
        audio_file = "Allen_yang_voice_original.wav"
        text_file = "Allen_yang_voice.txt"
    else:
        audio_file = "Allen_yang_voice_original.wav"
        text_file = "Allen_yang_voice.txt"

    audio_path = os.path.join(data_dir, audio_file)
    audio_text_path = os.path.join(data_dir, text_file)
    with open(audio_text_path, "r") as f:
        audio_text = f.read()
    audio_base64 = encode_base64_content_from_file(audio_path)
    print("Currently saying:", audio_text)
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
    # Use TTS server URL from configuration instead of hardcoded values
    client = OpenAI(base_url=settings.vllm_tts_url, api_key=settings.vllm_api_key)
    # Use configured model or auto-detect from server
    if settings.vllm_tts_model:
        model = settings.vllm_tts_model
    else:
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
