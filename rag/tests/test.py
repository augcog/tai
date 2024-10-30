import requests
import openai

# QUESTIONS:
questions = ["How can I rapidly become proficient in using the MoveIt Motion Planning Framework, specifically for the Franka Emika Panda robot, through the provided tutorials, and are there alternative robots that are already compatible with MoveIt? Please list them and provide me with a resource for further exploration. Additionally, if my custom robot is not yet supported by MoveIt, how can I integrate it into the framework using the tutorial section ""Integration with a New Robot""? Provide me with a step-by-step guide and necessary resources.",
             "How can I utilize the MoveGroup interface, available in both C++ and Python, to efficiently interact with MoveIt and access its various features through scripting, as described in the 'MoveGroup - ROS Wrappers in C++ and Python' section of this document? Please provide a step-by-step guide, including any necessary prerequisites and examples of how to use the interface to manipulate the robot's movements and configurations. Additionally, clarify any potential limitations or trade-offs of relying solely on the MoveGroup interface, and offer alternatives or complementary approaches for more advanced MoveIt applications.",
             "How can developers utilize MoveIt's C++ API to build more complex applications with significantly faster performance, and what are the potential benefits of bypassing the ROS Service/Action layers in this process? Please provide specific examples and use cases to illustrate the advantages of directly accessing MoveIt's C++ API.",
             "How can I implement time parameterization for motion planning in MoveIt! using the Time Parameterization tutorial provided in the documentation, and what are the benefits of using this technique in my robot's motion planning process? Additionally, can you provide examples of how to apply this technique in specific motion planning scenarios, and how to integrate it with other MoveIt! features such as the motion planning pipeline and planning scene? Finally, what are the system requirements for using time parameterization in MoveIt!, and how can I ensure optimal performance when implementing this technique in my robot's motion planning system?",
             "How can I integrate a new robot with MoveIt, and what tutorials are available in the 'Integration with a New Robot' section to help with this process? Provide a detailed explanation and list all the necessary steps involved in integrating a new robot with MoveIt, as well as any prerequisites or requirements that should be met before beginning the integration process. Additionally, describe how these tutorials can be accessed and navigated within the 'Integration with a New Robot' section."
             ]

# CONFIGS
get_chunks_url = "http://0.0.0.0:8000/api/chat/top_k_docs"
url = "http://128.32.43.233:8000/api/chat/completions"
API_KEY = ""
COURSE = "EE 106B"

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
    context_json = response.json()
    context = "\n".join(context_json['top_docs'])

    filled_prompt = QUERY_PROMPT.format(context=context, question=question)

    params = CompletionCreateParams(course=COURSE)
    params.add_message(role="user", content=filled_prompt)
    params_dict = params.to_dict()

    response = requests.post(url, json=params_dict)
    answer = response.text

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
    result = response.json()
    feedback = result['choices'][0]['message']['content']

    question_context_answer_feedback.append([question, context, answer, feedback])

print(question_context_answer_feedback)