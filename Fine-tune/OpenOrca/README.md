# Instructional Tuning Dataset Generator

## Overview
This script is designed to create a comprehensive dataset suitable for instructional tuning, leveraging OpenAI's GPT models. It aims to enrich the dataset not only with direct questions and answers but also with detailed explanations, making it ideal for educational and training purposes. The dataset is generated from a structured traversal of specified document directories, extracting text segments, and formulating questions based on these segments. Each question is then answered by the model, accompanied by an explanatory response to ensure a deeper understanding of the subject matter.

## Purpose
The primary goal of this dataset generator is to support instructional tuning processes, where the emphasis is not just on the correct answers but on the rationale behind them. This approach facilitates a more in-depth learning experience, catering to educational frameworks, AI training, and knowledge base enhancement.

## Features
- **Automated Question Generation**: Generates specific, long-form questions based on the content of the provided documents.
- **Detailed Explanations**: Alongside each answer, the model provides a detailed explanation tailored to enhance comprehension.
- **Customizable Token Limit**: Allows setting a maximum token limit for the generated text segments, ensuring compatibility with various model capacities.
## Usage
1. **Configuration**: Set the desired `model` and `token limit` within the script. Configure the `.env` file with your OpenAI API key.
   ```
   # TODO MODEL
   # model = 'zephyr'
   model = 'openai'
   # TODO TOKEN LIMIT
   n = 400
   # TODO Select Documents
   # docs = traverse_files("../scraper/Scrape_rst/Sawyer", "Sawyer")
   docs = traverse_files("../../rag/scraper/Scrape_pdf/textbook", "Robotics textbook")
   ```
2. **Execution**: Run the script with `python dataset_generator.py`. Specify the path to the documents directory and the desired start folder name within the script.
3. **Output**: The script generates a `CSV` file within the `Dataset` directory, containing columns for ID, Question, Document, and Response.

## Dataset Structure
- **ID**: A unique identifier constructed from the document's directory path and segment information.
- **Question**: The generated question based on the document segment.
- **Document**: The text segment from the document that the question is based on.
- **Response**: The model's answer to the question, accompanied by a detailed explanation.
