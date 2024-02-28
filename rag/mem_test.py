import openai
import json
from dotenv import load_dotenv
load_dotenv()

openai.api_key = "empty"
openai.api_base = "http://localhost:8000/v1"

str = "a"
count = 1000
token= 0
while(token < 1000):
    print(count)
    str=count*"a"
    b=openai.Embedding.create(model="text-embedding-ada-002", input=str)
    # print(b)
    # print(json.dumps(b, indent=2))
    count+=100
    token=b['usage']['total_tokens']
    print(f"token: {token}")

def chat_completion(system_message, human_message):
    messages=human_message+"\n---\n"+system_message

    completion = openai.Completion.create(model='gpt-3.5-turbo', prompt=messages, temperature=0, max_tokens=500)
    print(completion)
    # print(completion)

    # answer=completion['choices'][0]['message']["content"]
    answer=completion['choices'][0]['text']

    return answer