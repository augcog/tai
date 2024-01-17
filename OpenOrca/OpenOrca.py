from datasets import load_dataset
import openai
import pandas as pd
from dotenv import load_dotenv
import os

load_dotenv()

openai.api_key = os.getenv('OPENAI_API_KEY')
dataset = load_dataset("Open-Orca/OpenOrca", split = "train")

# Define a container for the new responses
updated_responses = []

# Create an empty data dictionary with the required columns
data = {
    'id': [],
    'system_prompt': [],
    'question': [],
    'response': []
}
# Number of data sets you want to train on
# Example: n=10
n=len(dataset)
selected_dataset=dataset[:n]
print(selected_dataset)
for i in range(n):
    system_prompt=selected_dataset['system_prompt'][i]
    question = selected_dataset['question'][i]
    messages = [
        {
            "role": "system",
            "content": system_prompt
        },
        {
            "role": "system",
            "content": question
        },
    ]
    # Generate samples using the API
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=messages,
        max_tokens=50  # Adjust based on your needs
    )
    print(response)
    # Extract response and append to your container
    data['id'].append(selected_dataset['id'][i])
    data['system_prompt'].append(system_prompt)
    data['question'].append(question)
    data['response'].append(response['choices'][0]['message']['content'].replace('\n\n','\n'))

# Convert the dictionary to a DataFrame
df = pd.DataFrame(data)
# Save the DataFrame to a CSV file
df.to_csv('updated_dataset.csv', index=False)



