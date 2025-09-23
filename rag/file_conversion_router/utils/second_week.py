from file_conversion_router.classes.page import Page
from file_conversion_router.classes.chunk import Chunk
from file_conversion_router.utils.title_handle import *
import os


def get_strutured_content_for_ipynb(
        md_content: str, file_name: str, course_name: str,
):
    title_list = get_title_list(md_content)
    load_dotenv()
    api_key = os.getenv("OPENAI_API_KEY")
    client = OpenAI(api_key=api_key)
    if not md_content.strip():
        raise ValueError("The content is empty or not properly formatted.")
    response = client.chat.completions.create(
        model="gpt-4.1",
        temperature=0.1,
        messages=[
            {
                "role": "system",
                "content": dedent(
                    f"""
                    ###  Extract and Create Problems
                    You are an expert in educational content creation, specifically for computer science courses. 
                    Your task is to create structured problems that can be used to assess students' understanding of the
                    exercises.
                    
                    For each problem:
                    - **ID:** Identify if this relates to an existing exercise in the material
                    - **Content:** Provide the main problem statement or scenario
                    - **Sub-problems:** Create exactly 2 sub-problems for each main problem:
                      - Each sub-problem should be a multiple choice question
                      - Options should include relevant key concepts from the material
                      - Provide clear explanations for correct answers

                    ### Guidelines:
                    - Focus on the most important concepts that students need to understand
                    - Ensure all content is directly derived from the provided markdown material
                    - Make problems challenging but fair, testing true comprehension
                    - Use key concepts as both correct answers and plausible distractors in multiple choice options
                    - Provide detailed explanations that reference the source material

                    Format your response as a valid JSON object matching the provided schema."""
                ),
            },
            {"role": "user", "content": f"{md_content}\n title_list: {title_list} "},
        ],
        response_format={
            'type': 'json_schema',
            'json_schema': {
                'name': 'course_content_knowledge_sorting',
                'strict': True,
                'schema': {
                    'type': 'object',
                    'properties': {
                        'problems': {
                            "type": "array",
                            "minItems" : len(title_list),
                            "maxItems": len(title_list),
                            'items': {
                                "type": "object",
                                "properties": {
                                    "ID": {
                                        "type": "string",
                                        'description': 'Which Exercise id this problem belongs to without any # and *, e.g. "Exercise 1","Challenge 1" '
                                    },
                                    'content': {
                                        'type': "string",
                                        'description': 'The whole content of Exercise or Challenge. Do not miss or change any part of the description.'
                                    },
                                    "sub_problem_1": {
                                        'type': "object",
                                        'properties': {
                                            'description_of_problem': {
                                                'type': 'string',
                                                'description': 'A multiple-choice question that better contains at least two correct options. Its purpose is to help students understand the problem statement and identify the underlying concepts. e.g. “What knowledge are involved in this problem?”'
                                            },
                                            'options': {
                                                'type': 'array',
                                                'items': {'type': 'string'},
                                                'description': 'The options for the sub problem, they should be key_concepts in the md_content'
                                            },
                                            'answers_options': {
                                                'type': 'array',
                                                'items': {'type': 'integer'},
                                                'description': 'The index of the option in the options array, e.g. [0,1] for the first and second options'
                                            },
                                            'explanation_of_answer': {
                                                'type': 'string',
                                                'description': 'The explanation of why this option is the answer, it should be a key_concept in the md_content'
                                            },
                                        },
                                        'required': ['description_of_problem', 'options', 'answers_options',
                                                     'explanation_of_answer'],
                                        'additionalProperties': False,
                                    },
                                    "sub_problem_2": {
                                        'type': "object",
                                        'properties': {
                                            'description_of_problem': {
                                                'type': 'string',
                                                'description': 'A multiple-choice question that better contain at least two correct options. Its purpose is to guide students in designing an approach or solution framework. e.g. “Which methods could be applied to solve this problem effectively?”'
                                            },
                                            'options': {
                                                'type': 'array',
                                                'items': {'type': 'string'},
                                                'description': 'The options for the sub problem, they should be key_concepts in the md_content'
                                            },
                                            'answers_options': {
                                                'type': 'array',
                                                'items': {'type': 'integer'},
                                                'description': 'The index of the option in the options array, e.g. [0,1] for the first and second options'
                                            },
                                            'explanation_of_answer': {
                                                'type': 'string',
                                                'description': 'The explanation of why this option is the answer, it should be a key_concept in the md_content'
                                            },
                                        },
                                        'required': ['description_of_problem', 'options', 'answers_options',
                                                     'explanation_of_answer'],
                                        'additionalProperties': False,
                                    },
                                },
                                'required': ['ID', 'content', 'sub_problem_1', 'sub_problem_2'],
                                'additionalProperties': False,
                            }
                        }
                    },
                    'required': ['problems'],
                    'additionalProperties': False,
                }
            }
        }
    )
    messages = response.choices[0].message
    data = messages.content
    content_dict = json.loads(data)
    return content_dict

def save_content_dict(content_dict: dict, output_path):
    """
    Save the structured content dictionary to a yaml file.

    Args:
        content_dict (dict): The structured content dictionary to save.
        output_path (str or Path): The path where the JSON file will be saved.

    """
    import yaml
    content_dict = process_problems(content_dict)
    with open(output_path, 'w', encoding='utf-8') as file:
        yaml.dump(content_dict, file, allow_unicode=True, default_flow_style=False)

def process_problems(content_dict):
    # Return just the list of problems, not a dictionary
    problems_list = []
    for problem in content_dict['problems']:
        processed_problem = {}
        processed_problem['problem_index'] = 1
        processed_problem['problem_id'] = problem['ID']
        processed_problem['problem_content'] = problem['content']

        # Create questions structure
        processed_problem['questions'] = {}
        for i in range(1, 3):
            question_key = f'sub_problem_{i}'
            sub_prob=problem[question_key]
            processed_problem['questions'][f'question_{i}'] = {
                'question': sub_prob.get('description_of_problem', ''),
                'choices': sub_prob.get('options', []),
                'answer': sub_prob.get('answers_options', []),
                'explanation': sub_prob.get('explanation_of_answer', '')
            }

        problems_list.append(processed_problem)

    return problems_list

if __name__ == "__main__":
    md_path = Path("/home/bot/bot/yk/YK_final/courses_out/ROAR Academy/Part Two/Week Two Exercises/Week Two Exercises.pdf.md")
    md_content = md_path.read_text(encoding="utf-8")
    file_name = md_path.stem
    course_name = "ROAR Academy"
    content_dict = get_strutured_content_for_ipynb(md_content, file_name, course_name)
    output_path = md_path.with_suffix('.yaml')
    save_content_dict(content_dict, output_path)
    print(f"Structured content saved to {output_path}")