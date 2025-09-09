from app.core.models.chat_completion import *
from app.services.rag_generation import build_memory_after_response
from typing import Union, AsyncIterator, List, Any, Dict
import re
import os
import base64
import io
import soundfile as sf
import numpy as np
from openai import OpenAI


async def chat_stream_parser(
        stream: AsyncIterator, reference_list: List[str], audio: bool = False, messages: List[Dict] = [],audio_text: str=None, engine: Any = None, old_sid: str = "", debug: bool = False
) -> AsyncIterator[str]:
    """
    Parse the streaming response from the chat model and yield deltas.
    """
    if audio_text:
        yield sse(AudioTranscript(text=audio_text))
    previous_text = ""
    text_seq = 0
    voice_seq=0
    audio_messages = []
    async for output in stream:
        text = output.outputs[0].text
        chunk = text[len(previous_text):]
        if not chunk:
            continue

        yield sse(ResponseDelta(seq=text_seq, text=chunk)); text_seq += 1
        print(chunk, end="")
        if audio:
            if '.' in chunk:

                # Find the last newline in text before chunk and get the text to be converted to audio after that
                last_newline_index = previous_text.rfind('.')
                audio_text = previous_text[last_newline_index + 1:] + chunk[:chunk.rfind('.') + 1]
                #replace all the consecutive \n with space no matter how many \n
                audio_text = re.sub(r'\n+', ' ', audio_text)
                # yield sse(ResponseDelta(seq=text_seq, text=audio_text)); text_seq += 1
                # print(audio_text, end="")
                if audio_text.strip()== "":
                    previous_text = text
                    continue
                messages_to_send= audio_text.split('.')
                for msg in messages_to_send:
                    if msg.strip():
                        audio_messages.append({"role": "user", "content": msg + '.'})
                        audio_iterator = audio_generator(audio_messages, stream=True)
                        audio_bytes_io = io.BytesIO()
                        async for data in audio_iterator:
                            yield sse(ResponseDelta(seq=voice_seq, audio_b64=data, audio_spec=AudioSpec())); voice_seq += 1
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
        previous_text = text
    else:
        audio_text = previous_text[previous_text.rfind('.') + 1:]
        # replace all the consecutive \n with space no matter how many \n
        # yield sse(ResponseDelta(seq=text_seq, text=audio_text)); text_seq += 1
        if audio_text.strip():
            print("\n[INFO] Final audio text:")
            print(audio_text, end="")
            audio_messages.append({"role": "user", "content": audio_text})
            audio_iterator = audio_generator(audio_messages, stream=True)
            audio_bytes_io = io.BytesIO()
            async for data in audio_iterator:
                yield sse(ResponseDelta(seq=voice_seq, audio_b64=data, audio_spec=AudioSpec())); voice_seq += 1
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
    pattern = re.compile(
        r'(?:\[Reference:\s*([\d,\s]+)\]'
        r'|\breference\s+(\d+(?:(?:\s*,\s*|\s*(?:and|&)\s*)\d+)*))',
        re.IGNORECASE
    )

    mentioned_references = {
        int(n)
        for m in pattern.finditer(previous_text)
        for n in re.findall(r'\d+', m.group(1) or m.group(2))
    }
    print(f"\n[INFO] Mentioned references: {mentioned_references}")
    references = []
    max_idx = len(reference_list)
    for i in sorted(mentioned_references):
        if 1 <= i <= max_idx:
            info_path, url, file_path = reference_list[i - 1]
            references.append(Reference(
                reference_idx=i,
                info_path=info_path,
                url=url,
                file_path=file_path
            ))
    if references:
        yield sse(ResponseReference(references=references))

    yield sse(Done())
    yield "data: [DONE]\n\n"  # Final done message for SSE clients

    await build_memory_after_response(messages, previous_text, references, engine, old_sid)

def encode_base64_content_from_file(file_path: str) -> str:
    """Encode a content from a local file to base64 format."""
    # Read the MP3 file as binary and encode it directly to Base64
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
    audio_text = re.sub(r'\[.*?\]', '', audio_text).replace('\n',' ').strip()
    print(f"[INFO] Audio text after cleaning: {audio_text}")
    return [{"role": "user", "content": audio_text}]

async def audio_generator(messages: List[Dict], stream: bool = True
) -> AsyncIterator[str]:
    """
    Parse the streaming response from the audio model and yield deltas.
    """
    data_dir = '/home/bot/localgpt/tai/ai_chatbot_backend/voice_prompts'
    # TODO: voice input by course
    # TODO: after file chat is enabled, use the previous voice from the video with index if possible to generate audio
    audio_path = os.path.join(data_dir, "trees_54.wav")
    audio_text_path = os.path.join(data_dir, "trees_54.txt")
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
    audio_bytes_io = io.BytesIO()
    if len(messages) > 3:
        messages = messages[:2] + messages[-1:]
    client = OpenAI(base_url='http://128.32.43.216:8000/v1',api_key='EMPTY')
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
        yield sse(ResponseDelta(seq=seq, audio_b64=data, audio_spec=AudioSpec())); seq += 1

    yield sse(Done())
    yield "data: [DONE]\n\n"  # Final done message for SSE clients