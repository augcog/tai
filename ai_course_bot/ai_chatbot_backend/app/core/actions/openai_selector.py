from openai import OpenAI
import os
from dotenv import load_dotenv
from app.core.models.chat_completion import Message as ROARChatCompletionMessage
from typing import List
from pydantic import BaseModel

load_dotenv()  # take environment variables from .env.

client = OpenAI(
    # This is the default and can be omitted
    api_key=os.environ.get("OPENAI_API_KEY"),
)


class Message(BaseModel):
    role: str
    content: str


def openai_formatter(messages: List[ROARChatCompletionMessage]) -> List[Message]:
    response: [Message] = []  # type: ignore
    for message in messages:
        response.append(Message(role=message.role, content=message.content))
    return response


def openai_selector(messages: List[Message], model="gpt-3.5-turbo", stream=True):
    stream = client.chat.completions.create(
        model=model,
        messages=messages,
        stream=stream,
    )
    return stream


def openai_parser(stream):
    for chunk in stream:
        result = chunk.choices[0].delta.content
        if result == None:
            yield ""
        else:
            yield chunk.choices[0].delta.content
