import requests
import openai
from dotenv import load_dotenv
import csv
import os
import pickle
import time

load_dotenv()

# QUESTIONS:
with open('/home/bot/bot/terrianne-eval/tai/rag/tests/zephyr_400_questions.pkl', 'rb') as file:
    data = pickle.load(file)

questions_all = [data[i][1] for i in range(len(data))]
questions = questions_all[:100]


# CONFIGS
get_chunks_url = "http://0.0.0.0:8000/api/chat/top_k_docs"
url = "http://0.0.0.0:8000/api/chat/completions"
COURSE = "EE 106B"
API_KEY = os.getenv("API_KEY")


# evaluates relevance of all chunks found during retrieval
EVALUATE_CHUNK_RELEVANCE = """###Task Description:
You are evaluating the reference context provided to a Retrieval-Augmented Generation-based LLM to answer a question related to the Berkeley course {course} asked by a student. You are given 5 chunks (pieces of context) that will be used by the LLM to generate a response to the student, as well as their corresponding cosine similarities (decimal between 0 and 1, representing how semantically similar the chunk is to the question calculated by a retrieval algorithm). You are responsible for scoring each chunk following a score rubric for the reference context, and answering several questions.
1. Write detailed feedback that assesses the quality of each of the 5 chunks, strictly based on the given score rubric, not evaluating in general. 
2. After writing feedback, write a score that is an integer between 1 and 5 per chunk. You should refer to the score rubric. 
3. The output format should look as follows: \”Individual chunk evaluation: [Chunk 1] {{write a feedback for criteria for chunk 1}} [Chunk 1 RESULT] {{an integer number between 1 and 5}}. [Chunk 2] {{write a feedback for criteria for chunk 2}} [Chunk 2 RESULT] {{an integer number between 1 and 5}}. [Chunk 3] {{write a feedback for criteria for chunk 3}} [Chunk 3 RESULT] {{an integer number between 1 and 5}}. [Chunk 4] {{write a feedback for criteria for chunk 4}} [Chunk 4 RESULT] {{an integer number between 1 and 5}}. [Chunk 5] {{write a feedback for criteria for chunk 5}} [Chunk 5 RESULT] {{an integer number between 1 and 5}}.”
4. Please do not generate any other opening, closing, and explanations.

###Chunks for reference:
{reference_context}

###Cosine similarities of chunks (in order of chunks in previous reference):
{cosine_sims}

###Question:
{instruction}

###Score Rubric:
[Is the chunk context relevant to the question?]
Score 1: The chunk is completely insufficient in answering the question.
Score 2: The chunk is mostly insufficient in answering the question.
Score 3: The chunk is somewhat sufficient in answering the question.
Score 4: The chunk is mostly sufficient in answering the question.
Score 5: The chunk is completely sufficient in answering the question.
"""


# evaluates quality of all chunks found during retrieval
EVALUATE_CHUNK_QUALITY = """###Task Description:
You are evaluating the reference context provided to a Retrieval-Augmented Generation-based LLM to answer a question related to the Berkeley course {course} asked by a student. You are given 5 chunks (pieces of context) that will be used by the LLM to generate a response to the student, as well as their corresponding cosine similarities (decimal between 0 and 1, representing how semantically similar the chunk is to the question). You are responsible for scoring each chunk following a score rubric for the reference context, and answering several questions.
1. Write detailed feedback that assesses the quality of each of the 5 chunks, strictly based on the given score rubric, not evaluating in general. 
2. After writing feedback, write a score that is an integer between 1 and 5 per chunk. You should refer to the score rubric. 
3. The output format should look as follows: \”Individual chunk evaluation: [Chunk 1] {{write a feedback for criteria for chunk 1}} [Chunk 1 RESULT] {{an integer number between 1 and 5}}. [Chunk 2] {{write a feedback for criteria for chunk 2}} [Chunk 2 RESULT] {{an integer number between 1 and 5}}. [Chunk 3] {{write a feedback for criteria for chunk 3}} [Chunk 3 RESULT] {{an integer number between 1 and 5}}. [Chunk 4] {{write a feedback for criteria for chunk 4}} [Chunk 4 RESULT] {{an integer number between 1 and 5}}. [Chunk 5] {{write a feedback for criteria for chunk 5}} [Chunk 5 RESULT] {{an integer number between 1 and 5}}.”
4. Please do not generate any other opening, closing, and explanations.

###Chunks for reference:
{reference_context}

###Cosine similarities of chunks (in order of chunks in previous reference):
{cosine_sims}

###Question:
{instruction}

###Score Rubric:
[Does the chunk provide clear information and how well does it read?]
Score 1: The chunk is completely unclear and does not read well.
Score 2: The chunk is mostly unclear and does not read well.
Score 3: The chunk is somewhat sufficient in clarity and reads alright.
Score 4: The chunk is mostly sufficient in clarity and reads well.
Score 5: The chunk is completely sufficient in clarity and reads well.
"""


# evaluates ability to find correct main ideas in context to create relevant response
EVALUATE_MODEL = """###Task Description:
You are evaluating the reference context provided to a Retrieval-Augmented Generation-based LLM to answer a question related to the Berkeley course {course} asked by a student. You are given 5 chunks (pieces of context) that will be used by the LLM to generate a response to the student, and the response generated by an LLM. You are responsible for scoring the response following a score rubric for the response.
1. Write detailed feedback that assesses this quality of the response strictly based on the given score rubric, not evaluating in general. Then, answer the following question: Given these chunks used as context for the LLM-generated response, does the LLM have enough general background to connect the dots between the chunks? (Answer YES or NO, and explain)
2. After writing feedback, write a score that is an integer between 1 and 5. You should refer to the score rubric.
3. The output format should look as follows: \"Response feedback: {{write a feedback for criteria}} [RESULT] {{an integer number between 1 and 5}}. Question: {{YES / NO}}, {{explanation}}."
4. Please do not generate any other opening, closing, and explanations. Be sure to include [RESULT] in your output.

###Chunks for reference:
{reference_context}

###Question:
{instruction}

###Response to evaluate:
{response}

###Score Rubric:
[Does the response use the main ideas of the reference context to form a relevant response?]
Score 1: The response does not at all use the main ideas of the reference context to form a relevant response. 
Score 2: The response hardly uses the main ideas of the reference context to form a relevant response.
Score 3: The response somewhat uses the main ideas of the reference context to form a relevant response.
Score 4: The response mostly uses the main ideas of the reference context to form a relevant response.
Score 5: The response completely uses the main ideas of the reference context to form a relevant response.
"""

# evaluates overall factualness of response to a question and helpfulness to student
# see if it is giving good answer to the question -- still provide chunks for reference. want to show as review of TAI, score based on helpfulness
EVALUATE_RESPONSE = """###Task Description:
You are evaluating the reference context provided to a Retrieval-Augmented Generation-based LLM to answer a question related to the Berkeley course {course} asked by a student. You are responsible for scoring the response following a score rubric for the response.
1. Write detailed feedback that assesses this quality of the response strictly based on the given score rubric, not evaluating in general.
2. After writing feedback, write a score that is an integer between 1 and 5. You should refer to the score rubric.
3. The output format should look as follows: \"Response feedback: {{write a feedback for criteria}} [RESULT] {{an integer number between 1 and 5}}."
4. Please do not generate any other opening, closing, and explanations. Be sure to include [RESULT] in your output.

###Chunks for reference:
{reference_context}

###Question:
{instruction}

###Response to evaluate:
{response}

###Score Rubric:
[Based on the chunks for reference and the question, is the response helpful for the student overall?]
Score 1: The response is not helpful to the student.
Score 2: The response is mostly not helpful to the student.
Score 3: The response is somewhat helpful to the student.
Score 4: The response is mostly helpful to the student.
Score 5: The response is completely helpful to the student.
"""

QUERY_PROMPT = """
Answer the following question as accurately as possible.
Question: {question}
"""

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

def query_tai_llm(messages, model="gpt-4-turbo", temperature=0.7):
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

question_context_answer_feedback = []

for i, question in enumerate(questions):
    params = {
        "message": question,
        "k": 5,
        "course": COURSE
    }

    response = requests.post(get_chunks_url, params=params)
    context_json = response.json()
    context = "\n".join(context_json['top_docs'])
    used_chunks = context_json['used_chunks']
    distances = context_json['distances']

    filled_prompt = QUERY_PROMPT.format(question=question)

    params = CompletionCreateParams(course=COURSE)
    params.add_message(role="user", content=filled_prompt)
    params_dict = params.to_dict()

    response = requests.post(url, json=params_dict)

    answer = response.text

    prompt1 = EVALUATE_CHUNK_RELEVANCE.format(
        course=COURSE,
        instruction=question,
        reference_context=context,
        cosine_sims=distances,
    )

    headers1 = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }

    data1 = {
        'model': 'gpt-4',
        'messages': [{'role': 'user', 'content': prompt1}],
        'temperature': 0
    }

    for _ in range(3):
        try:
            response1 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers1, json=data1)
            result1 = response1.json()
            break
        except requests.exceptions.ConnectionError as e:
            print("Connection error, retrying in 5 seconds")
            time.sleep(5)
    feedback1 = result1['choices'][0]['message']['content'] if result1 else "No response"

    prompt2 = EVALUATE_CHUNK_QUALITY.format(
        course=COURSE,
        instruction=question,
        reference_context=context,
        cosine_sims=distances,
    )

    headers2 = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }

    data2 = {
        'model': 'gpt-4',
        'messages': [{'role': 'user', 'content': prompt2}],
        'temperature': 0
    }

    for _ in range(3):
        try:
            response2 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers2, json=data2)
            result2 = response2.json()
            break
        except requests.exceptions.ConnectionError as e:
            print("Connection error, retrying in 5 seconds")
            time.sleep(5)
    feedback2 = result2['choices'][0]['message']['content'] if result2 else "No response"

    prompt3 = EVALUATE_MODEL.format(
        course=COURSE,
        instruction=question,
        response=answer,
        reference_context=context,
    )

    headers3 = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }

    data3 = {
        'model': 'gpt-4',
        'messages': [{'role': 'user', 'content': prompt3}],
        'temperature': 0
    }

    for _ in range(3):
        try:
            response3 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers3, json=data3)
            result3 = response3.json()
            break
        except requests.exceptions.ConnectionError as e:
            print("Connection error, retrying in 5 seconds")
            time.sleep(5)
    feedback3 = result3['choices'][0]['message']['content'] if result3 else "No response"

    prompt4 = EVALUATE_RESPONSE.format(
        course=COURSE,
        instruction=question,
        response=answer,
        reference_context=context,
    )

    headers4 = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }

    data4 = {
        'model': 'gpt-4',
        'messages': [{'role': 'user', 'content': prompt4}],
        'temperature': 0
    }

    for _ in range(3):
        try:
            response4 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers4, json=data4)
            result4 = response4.json()
            break
        except requests.exceptions.ConnectionError as e:
            print("Connection error, retrying in 5 seconds")
            time.sleep(5)
    feedback4 = result4['choices'][0]['message']['content'] if result4 else "No response"

    question_context_answer_feedback.append([question, context, answer, used_chunks, distances, feedback1, feedback2, feedback3, feedback4])
    print("Done evaluating question: " + str(i))

output_file = "question_context_feedback.csv"

with open(output_file, mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)

    # Write the header
    writer.writerow(
        ["Question", "Context", "Answer", "Used Chunks", "Cosine Distances", "Feedback1", "Feedback2", "Feedback3",
         "Feedback4"])

    # Write the data
    writer.writerows(question_context_answer_feedback)

print(f"Data written to {output_file}")