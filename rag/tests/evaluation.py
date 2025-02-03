import requests
import openai
from dotenv import load_dotenv
import csv
import os

load_dotenv()

# QUESTIONS:
questions = ["How can I rapidly become proficient in using the MoveIt Motion Planning Framework, specifically for the Franka Emika Panda robot, through the provided tutorials, and are there alternative robots that are already compatible with MoveIt? Please list them and provide me with a resource for further exploration. Additionally, if my custom robot is not yet supported by MoveIt, how can I integrate it into the framework using the tutorial section ""Integration with a New Robot""? Provide me with a step-by-step guide and necessary resources.",
             "How can I utilize the MoveGroup interface, available in both C++ and Python, to efficiently interact with MoveIt and access its various features through scripting, as described in the 'MoveGroup - ROS Wrappers in C++ and Python' section of this document? Please provide a step-by-step guide, including any necessary prerequisites and examples of how to use the interface to manipulate the robot's movements and configurations. Additionally, clarify any potential limitations or trade-offs of relying solely on the MoveGroup interface, and offer alternatives or complementary approaches for more advanced MoveIt applications.",
             "How can developers utilize MoveIt's C++ API to build more complex applications with significantly faster performance, and what are the potential benefits of bypassing the ROS Service/Action layers in this process? Please provide specific examples and use cases to illustrate the advantages of directly accessing MoveIt's C++ API.",
             "How can I implement time parameterization for motion planning in MoveIt! using the Time Parameterization tutorial provided in the documentation, and what are the benefits of using this technique in my robot's motion planning process? Additionally, can you provide examples of how to apply this technique in specific motion planning scenarios, and how to integrate it with other MoveIt! features such as the motion planning pipeline and planning scene? Finally, what are the system requirements for using time parameterization in MoveIt!, and how can I ensure optimal performance when implementing this technique in my robot's motion planning system?",
             "How can I integrate a new robot with MoveIt, and what tutorials are available in the 'Integration with a New Robot' section to help with this process? Provide a detailed explanation and list all the necessary steps involved in integrating a new robot with MoveIt, as well as any prerequisites or requirements that should be met before beginning the integration process. Additionally, describe how these tutorials can be accessed and navigated within the 'Integration with a New Robot' section."
            ]

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

for question in questions:
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

    response1 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers1, json=data1)
    result1 = response1.json()
    feedback1 = result1['choices'][0]['message']['content']

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

    response2 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers2, json=data2)
    result2 = response2.json()
    feedback2 = result2['choices'][0]['message']['content']

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

    response3 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers3, json=data3)
    result3 = response3.json()
    feedback3 = result3['choices'][0]['message']['content']

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

    response4 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers4, json=data4)
    result4 = response4.json()
    feedback4 = result4['choices'][0]['message']['content']

    question_context_answer_feedback.append([question, context, answer, used_chunks, distances, feedback1, feedback2, feedback3, feedback4])

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


# analyzing results afterward for response 1-- 2 step retrival, 1. ranking, 2. threshold 
# - if documents are irrelavent
# - no documents over similarity threshold -- threshold is too high or no supported documents
# need to classify into these two items 

# response 2: 
# - irrelevant documents in context (yes / no) based on score

#+++++++++++++++++++++++++++++++++

#another round of conversation
# need to return ranked version of all documents from route (including ones below the threshold) to ask LLM to evaluate if they are relevant or not

# is the prompt given to llama3 good or not -- see if gpt4 can evaluate 
# -- too long prompt -- attention not good
# -- too short prompt -- not enough information to describe situation



# dec 5th
# give example ? of rating per each level of prompt
# take larger k
# add cosine similarity of each chunk 
# one score per chunk 
# get top 5 k, then evaluate if this is better or worse than top 3 --> add this into the prompt 

# just parse the score out (quantitative answer)

# in the gpt response, explain which one of these is the problem 
# several options: improve on retrieval accuracy, clean up the chunk, etc. 
# give improvement advice -- for the second two 

# is the chunk bad to read and is the information clear (new prompt)

# merge prompt one and two, want this and current third prompt to be advice for how to improve retrieval 
# current 3rd prompt -- does model have general background to connect the dots between the chunks (add this in the advice output)
# current 4th prompt should be how good the response would be for a student

# compare trends of our scores to other benchmarks / leaderboards

# structured api call for gpt to choose from list of advice 

# milestone: auto-pipeline, take in ed forum file. 


#1. retrieval accuracy used to get the best retrieval method (embedding model, reranking model, retrieval algorithms)
#2. about picking the best LLM for TAI (ex. llama3, deepseek, changing the prompt / giving more exmaples for q and a)
#3 not used to make adjustments on the method and overall score we want to show to peope who care about performance

# fix prompts, share results, read papers

# how to prepare a good dataset for pipeline for any classes --> ed pipeline, automated question generation pipeline, real test cases 
# read code for question_generator.py