import csv
import requests
from typing import List
import openai

class CompletionCreateParams:
    def __init__(self, course, messages=None, stream=True, temperature=5):
        self.course = course
        self.messages = messages if messages is not None else []
        self.stream = stream
        self.temperature = temperature

    def add_message(self, role, content):
        self.messages.append({"role": role, "content": content})

    def to_dict(self):
        return {
            "course": self.course,
            "messages": self.messages,
            "stream": self.stream,
            "temperature": self.temperature
        }

url = "http://128.32.43.233:8000/api/chat/completions"
API_KEY = ""

EVALUATION_PROMPT = """###Task Description:
An instruction (might include an Input inside it), a response to evaluate, a reference answer that gets a score of 5, and a score rubric representing a evaluation criteria are given.
1. Write a detailed feedback that assess the quality of the response strictly based on the given score rubric, not evaluating in general.
2. After writing a feedback, write a score that is an integer between 1 and 5. You should refer to the score rubric.
3. The output format should look as follows: \"Feedback: {{write a feedback for criteria}} [RESULT] {{an integer number between 1 and 5}}\"
4. Please do not generate any other opening, closing, and explanations. Be sure to include [RESULT] in your output.

###The instruction to evaluate:
{instruction}

###Response to evaluate:
{response}

###Reference context:
{reference_context}

###Feedback:

###Score Rubrics:
[Is the response correct, accurate, and factual based on the reference answer?]
Score 1: The response is completely incorrect, inaccurate, and/or not factual.
Score 2: The response is mostly incorrect, inaccurate, and/or not factual.
Score 3: The response is somewhat correct, accurate, and/or factual.
Score 4: The response is mostly correct, accurate, and factual.
Score 5: The response is completely correct, accurate, and factual.
"""

QUERY_PROMPT = """
Context: {context}
Question: {question}
Based on the context provided, answer the question as accurately as possible.
"""

# Parse question and context from questions_context.csv
def read_csv_to_tuples(file_path):
    question_context_pairs = []

    with open(file_path, mode='r', encoding='utf-8') as file:
        reader = csv.reader(file)

        next(reader)

        for row in reader:
            if len(row) >= 2:
                question = row[0]
                context = row[1]
                question_context_pairs.append([question, context])

    return question_context_pairs

def query_chat_llm(messages, model="gpt-4-turbo", temperature=0.7):
    try:
        response = openai.ChatCompletion.create(
            model=model,
            messages=messages,
            temperature=temperature
        )
        # Extract and return the generated message
        return response['choices'][0]['message']['content'].strip()
    except Exception as e:
        return f"Error occurred: {e}"

file_path = '/Users/terriannezhang/Desktop/tai/tai/rag/tests/questions_context.csv'
question_context_pairs = read_csv_to_tuples(file_path)

# Iterate through pairs in question_context_pairs
for qc_pair in question_context_pairs:
    question, context = qc_pair
    filled_prompt = QUERY_PROMPT.format(context=context, question=question)

    params = CompletionCreateParams(course="ee106b")

    params.add_message(role="user", content=filled_prompt)

    params_dict = params.to_dict()

    response = requests.post(url, json=params_dict)
    if response.ok:
        print("Question:")
        print(response.text)
        qc_pair.append(response.text)
    else:
        print(response.status_code, response.text)

for question, context, answer in question_context_pairs:
    prompt = EVALUATION_PROMPT.format(
        instruction=question,
        response=answer,
        reference_context=context,
    )

    headers = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }

    data = {
        'model': 'gpt-4',
        'messages': [{'role': 'user', 'content': prompt}],
        'temperature': 0
    }

    response = requests.post('https://api.openai.com/v1/chat/completions', headers=headers, json=data)

    if response.status_code == 200:
        result = response.json()
        feedback = result['choices'][0]['message']['content']
        print(feedback)
    else:
        print(f"Error: {response.status_code}, {response.text}")

# --> chunk size (right now chunk size is cut in the middle, might retrieve document split in half) --> chunking method?
# --> context --> is this context enough information to answer question --> better to do for all k chunks retrieved 
# --> context --> is the context relevant to the question
# --> RAG test results google sheets file

# --> configure different ways to test retrieval, embedding 
# 2 files -- 1st one is agent (endpoint to get retrieval results + how they are ranked) -- 2nd one is 




