import requests
import openai
import csv

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
API_KEY = ""
COURSE = "EE 106B"


# evaluates all chunks found during retrieval
EVALUATION_PROMPT_FOUND_CONTEXT = """###Task Description:
You are evaluating the reference context provided to a RAG-based LLM to answer a question related to the Berkeley course {course}. You are responsible for scoring the response following a score rubric for the reference context. 
1. Write detailed feedback that assesses this quality of the reference context strictly based on the given score rubric, not evaluating in general.
2. After writing feedback, write a score that is an integer between 1 and 5. You should refer to the score rubric.
3. The output format should look as follows: \"Context sufficiency feedback (found): {{write a feedback for criteria}} [RESULT] {{an integer number between 1 and 5}}."
4. Please do not generate any other opening, closing, and explanations. Be sure to include all 3 [RESULT]s in your output.

###Reference context to evaluate:
{reference_context}

###Question:
{instruction}

###Feedback:

###Score Rubric:
[Is the reference context relevant to the question?] 
Score 1: The reference context is completely insufficient to answer the question.
Score 2: The reference context is mostly insufficient to answer the question.
Score 3: The reference context is somewhat sufficient to answer the question.
Score 4: The reference context is mostly sufficient to answer the question.
Score 5: The reference context is completely sufficient to answer the question.
"""

# evaluates the chunks with higher similarity than 0.45 used for the response
EVALUATION_PROMPT_USED_CONTEXT = """###Task Description:
You are evaluating the reference context provided to a RAG-based LLM to answer a question related to the Berkeley course {course}. You are responsible for scoring the response following a score rubric for the reference context. 
1. Write detailed feedback that assesses this quality of the reference context strictly based on the given score rubric, not evaluating in general.
2. After writing feedback, write a score that is an integer between 1 and 5. You should refer to the score rubric.
3. The output format should look as follows: \"Context sufficiency feedback (used): {{write a feedback for criteria}} [RESULT] {{an integer number between 1 and 5}}."
4. Please do not generate any other opening, closing, and explanations. Be sure to include all 3 [RESULT]s in your output.

###Reference context to evaluate:
{reference_context}

###Question:
{instruction}

###Feedback:

###Score Rubric:
[Is the reference context relevant to the question?] 
Score 1: The reference context is completely insufficient to answer the question.
Score 2: The reference context is mostly insufficient to answer the question.
Score 3: The reference context is somewhat sufficient to answer the question.
Score 4: The reference context is mostly sufficient to answer the question.
Score 5: The reference context is completely sufficient to answer the question.
"""

# evaluates ability to find correct main ideas in context to create relevant response
EVALUATION_PROMPT_RESPONSE = """###Task Description:
You are evaluating the response of a RAG-based LLM based on a question related to the Berkeley course {course} and a reference context. You are responsible for scoring the response following a score rubric for the response. 
1. Write detailed feedback that assesses this quality of the response strictly based on the given score rubric, not evaluating in general.
2. After writing feedback, write a score that is an integer between 1 and 5. You should refer to the score rubric.
3. The output format should look as follows: \"Response feedback: {{write a feedback for criteria}} [RESULT] {{an integer number between 1 and 5}}."
4. Please do not generate any other opening, closing, and explanations. Be sure to include all 3 [RESULT]s in your output.

###Reference context:
{reference_context}

###Question:
{instruction}

###Response to evaluate:
{response}

###Feedback:

###Score Rubric:
[Does the response use the main ideas the reference context to form a relevant response?]
Score 1: The response does not at all use the main ideas of the reference context to form a relevant response.  
Score 2: The response hardly uses the main ideas of the reference context to form a relevant response. 
Score 3: The response somewhat uses the main ideas of the reference context to form a relevant response. 
Score 4: The response mostly uses the main ideas of the reference context to form a relevant response. 
Score 5: The response completely uses the main ideas of the reference context to form a relevant response. 
"""

# evaluates overall factualness of response to a question
EVALUATION_PROMPT_OVERALL = """###Task Description:
You are evaluating the quality of the response of a RAG-based LLM based on a question related to the Berkeley course {course}. You are responsible for scoring the response following a score rubric for the response. 
1. Write detailed feedback that assesses this quality of the response strictly based on the given score rubric, not evaluating in general.
2. After writing feedback, write a score that is an integer between 1 and 5. You should refer to the score rubric.
3. The output format should look as follows: \"Response feedback: {{write a feedback for criteria}} [RESULT] {{an integer number between 1 and 5}}."
4. Please do not generate any other opening, closing, and explanations. Be sure to include all 3 [RESULT]s in your output.

###Question:
{instruction}

###Response to evaluate:
{response}

###Feedback:

###Score Rubric:
[Is the response correct, relevant, accurate, and factual overall?]
Score 1: The response is completely incorrect, inaccurate, and/or not factual.
Score 2: The response is mostly incorrect, inaccurate, and/or not factual.
Score 3: The response is somewhat correct, accurate, and/or factual.
Score 4: The response is mostly correct, accurate, and factual.
Score 5: The response is completely correct, accurate, and factual.
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
        "k": 1,
        "course": COURSE
    }

    response = requests.post(get_chunks_url, params=params)
    print(response.text)
    context_json = response.json()
    context = "\n".join(context_json['top_docs'])
    found_chunks = context_json['found_chunks']
    used_chunks = context_json['used_chunks']


    filled_prompt = QUERY_PROMPT.format(question=question)

    params = CompletionCreateParams(course=COURSE)
    params.add_message(role="user", content=filled_prompt)
    params_dict = params.to_dict()

    response = requests.post(url, json=params_dict)

    answer = response.text

    prompt1 = EVALUATION_PROMPT_FOUND_CONTEXT.format(
        course=COURSE,
        instruction=question,
        reference_context=context,
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

    prompt2 = EVALUATION_PROMPT_USED_CONTEXT.format(
        course=COURSE,
        instruction=question,
        reference_context=context,
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

    prompt3 = EVALUATION_PROMPT_RESPONSE.format(
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

    prompt4 = EVALUATION_PROMPT_OVERALL.format(
        course=COURSE,
        instruction=question,
        response=answer,
    )

    headers4 = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }

    data4 = {
        'model': 'gpt-4',
        'messages': [{'role': 'user', 'content': prompt3}],
        'temperature': 0
    }

    response4 = requests.post('https://api.openai.com/v1/chat/completions', headers=headers3, json=data3)
    result4 = response3.json()
    feedback4 = result3['choices'][0]['message']['content']

    question_context_answer_feedback.append([question, context, answer, found_chunks, used_chunks, feedback1, feedback2, feedback3, feedback4])

output_file = "question_context_feedback.csv"

with open(output_file, mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)

    # Write the header
    writer.writerow(
        ["Question", "Context", "Answer", "Found Chunks", "Used Chunks", "Feedback1", "Feedback2", "Feedback3",
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