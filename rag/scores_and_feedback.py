import os
from dotenv import load_dotenv
import openai
import pandas as pd
import numpy as np
import pickle

# Ensure that the .env file is in the correct directory
dotenv_path = os.path.join(os.path.dirname(__file__), '.env')
load_dotenv(dotenv_path=dotenv_path)

# Check if the environment variable is loaded correctly
api_key = os.getenv('OPENAI_API_KEY')
if not api_key:
    raise ValueError("OpenAI API key is not set. Please set it in the .env file or directly in the script.")
openai.api_key = api_key

# Function to load data from pickle files
def load_data(directory):
    data = []
    for filename in os.listdir(directory):
        if filename.endswith('.pkl'):
            with open(os.path.join(directory, filename), 'rb') as file:
                data.append(pickle.load(file))
    return data

# Placeholder for generating a response based on top 3 segments
def generate_local_model_response(top_3_segments):
    return "Generated response based on the top 3 segments."

# Function to get ratings and feedback using OpenAI API
def get_rating_and_feedback(segment, question):
    prompt = f"""
    Question: {question}

    Segment: {segment}

    Please rate the relevance of the segment on a scale from 1 to 5 based on its accuracy, completeness, and helpfulness. Then, provide constructive feedback on how the segment could be improved.

    Rating: 
    Feedback: 
    """
    completion = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are an expert evaluator."},
            {"role": "user", "content": prompt}
        ],
        max_tokens=150,
        temperature=0
    )
    content = completion.choices[0].message["content"].strip()
    print(f"DEBUG: API Response: {content}")  # Debug print

    rating = None
    feedback = "No valid feedback received"

    if "Rating:" in content and "Feedback:" in content:
        try:
            rating_part, feedback_part = content.split("Feedback:")
            rating = rating_part.split("Rating:")[1].strip()
            feedback = feedback_part.strip()
        except Exception as e:
            print(f"Error parsing the response: {e}")

    try:
        rating = int(rating.split('/')[0])
    except:
        rating = None

    return rating if rating is not None else 0, feedback

# Process and save the results
def process_and_save_results(data):
    results = []
    for entry in data:
        print(f"Processing entry: {entry}")  # Debug print to check entry structure
        if 'doc' not in entry:
            print(f"Missing 'doc' key in entry: {entry}")
            continue

        top_3_segments = entry['doc'][:3]
        response = generate_local_model_response(top_3_segments)

        ratings_feedback = [get_rating_and_feedback(segment, entry['question']) for segment in top_3_segments]

        aggregated_rating = np.mean([rating for rating, _ in ratings_feedback])

        consolidated_feedback = " | ".join([feedback for _, feedback in ratings_feedback])

        results.append({
            "question": entry['question'],
            "segment_1": top_3_segments[0] if len(top_3_segments) > 0 else "",
            "segment_2": top_3_segments[1] if len(top_3_segments) > 1 else "",
            "segment_3": top_3_segments[2] if len(top_3_segments) > 2 else "",
            "response": response,
            "aggregated_rating": aggregated_rating,
            "feedback": consolidated_feedback
        })

    results_df = pd.DataFrame(results)
    results_csv_file = "dataset.csv"
    results_df.to_csv(results_csv_file, index=False)
    print(f"Results with ratings and feedback have been written to {results_csv_file}")

# Main execution
data_directory = 'question_set'
data = load_data(data_directory)

print("Loaded data:", data)

process_and_save_results(data)
