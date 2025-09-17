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


def extract_channels(text: str) -> dict:
    # 1) Remove the special marker wherever it appears
    cleaned = re.sub(r"<\|start\|\>assistant\s*", "", text)

    # 2) Capture channel/message pairs; message ends at next channel, <|end|>, or end-of-text
    pattern = re.compile(
        r"<\|channel\|\>(?P<channel>[A-Za-z0-9_]+)\s*"
        r"<\|message\|\>(?P<message>.*?)(?=(?:<\|channel\|\>|<\|end\|\>|\Z))",
        re.DOTALL
    )

    result = {}
    for m in pattern.finditer(cleaned):
        ch = m.group("channel").strip()
        msg = m.group("message").strip()
        result[ch] = msg  # if duplicate channels appear, the last one wins
    return result

async def chat_stream_parser(
        stream: AsyncIterator, reference_list: List[str], audio: bool = False, messages: List[Message]= None,audio_text: str=None, engine: Any = None, old_sid: str = "", course_code: str = None, debug: bool = False
) -> AsyncIterator[str]:
    """
    Parse the streaming response from the chat model and yield deltas.
    """
    if audio_text:
        yield sse(AudioTranscript(text=audio_text))
    previous_channels = {}
    previous_index = -2
    text_seq = 0
    voice_seq=0
    audio_messages = []
    async for output in stream:
        text = output.outputs[0].text
        channels= extract_channels(text)
        if not channels:
            continue
        chunks= {c: channels[c][len(previous_channels.get(c,"")):] for c in channels if channels[c] != previous_channels.get(c,"")}
        if not chunks:
            continue
        for channel in chunks:
            chunk = chunks[channel]
            if not chunk.strip():
                continue
            yield sse(ResponseDelta(seq=text_seq, text_channel=channel, text=chunk)); text_seq += 1
            print(chunk, end="")
        previous_channels = channels
        if audio and 'final' in channels:
            last_newline_index = channels['final'].rfind('. ')
            if last_newline_index >previous_index+2:
                audio_text = channels['final'][previous_index + 2:last_newline_index+2]
                previous_index = last_newline_index
                #replace all the consecutive \n with space no matter how many \n
                # audio_text = re.sub(r'\n+', ' ', audio_text)
                if audio_text.strip()== "":
                    continue
                messages_to_send= audio_text.split('. ')
                for msg in messages_to_send:
                    if msg.strip():
                        audio_messages.append({"role": "user", "content": msg + '. '})
                        print("\n[INFO] Audio text:")
                        print(msg + '. ')
                        speaker_name = get_speaker_name(course_code)
                        audio_iterator = audio_generator(audio_messages, stream=True, speaker_name=speaker_name)
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
    else:
        if audio and 'final' in channels:
            audio_text = channels['final'][previous_index + 2:]
            # replace all the consecutive \n with space no matter how many \n
            # yield sse(ResponseDelta(seq=text_seq, text=audio_text)); text_seq += 1
            if audio_text.strip():
                messages_to_send = audio_text.split('. ')
                for msg in messages_to_send:
                    if msg.strip():
                        audio_messages.append({"role": "user", "content": msg + '. '})
                        print("\n[INFO] Audio text:")
                        print(msg + '. ')
                        speaker_name= get_speaker_name(course_code)
                        audio_iterator = audio_generator(audio_messages, stream=True, speaker_name=speaker_name)
                        audio_bytes_io = io.BytesIO()
                        async for data in audio_iterator:
                            yield sse(ResponseDelta(seq=voice_seq, audio_b64=data, audio_spec=AudioSpec(),speaker_name=speaker_name));
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

    # # convert token ids to text
    # print("\n[INFO] Full response text:")
    # from transformers import AutoTokenizer
    # TOKENIZER_MODEL_ID = "openai/gpt-oss-20b"
    # TOKENIZER = AutoTokenizer.from_pretrained(TOKENIZER_MODEL_ID)
    # full_response_text = TOKENIZER.decode(token, skip_special_tokens=False)
    # print(full_response_text)


    pattern = re.compile(
        r'(?:\[Reference:\s*([\d,\s]+)\]'
        r'|\breference\s+(\d+(?:(?:\s*,\s*|\s*(?:and|&)\s*)\d+)*))',
        re.IGNORECASE
    )

    mentioned_references = {
        int(n)
        for m in pattern.finditer(channels['final'])
        for n in re.findall(r'\d+', m.group(1) or m.group(2))
    }
    print(f"\n[INFO] Mentioned references: {mentioned_references}")
    references = []
    max_idx = len(reference_list)
    for i in sorted(mentioned_references):
        if 1 <= i <= max_idx:
            info_path, url, file_uuid,chunk_index = reference_list[i - 1]
            references.append(Reference(
                reference_idx=i,
                info_path=info_path,
                url=url,
                file_uuid=file_uuid,
                chunk_index=chunk_index
            ))
    if references:
        yield sse(ResponseReference(references=references))

    yield sse(Done())
    yield "data: [DONE]\n\n"  # Final done message for SSE clients

    await build_memory_after_response(messages, channels['final'], references, engine, old_sid)

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
    data_dir = '/home/bot/localgpt/tai/ai_chatbot_backend/voice_prompts'
    
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