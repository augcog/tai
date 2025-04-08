import os
import csv
from openai import OpenAI
from dotenv import load_dotenv
import tiktoken

# Load API key from .env file
load_dotenv()
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def split_into_chunks(text, max_tokens=4000):
    encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
    words = text.split("\n\n")  # simple chunking by paragraph
    chunks, current_chunk, current_tokens = [], [], 0

    for paragraph in words:
        tokens = len(encoding.encode(paragraph))
        if current_tokens + tokens > max_tokens:
            chunks.append("\n\n".join(current_chunk))
            current_chunk, current_tokens = [paragraph], tokens
        else:
            current_chunk.append(paragraph)
            current_tokens += tokens

    if current_chunk:
        chunks.append("\n\n".join(current_chunk))

    return chunks

def find_all_markdown_files(root_dir):
    """
    Recursively find all .md files in a tree of folders.
    """
    md_files = []
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".md"):
                md_files.append(os.path.join(root, file))
    return md_files

def read_markdown_file(md_path):
    """
    Read the contents of the markdown file.
    """
    with open(md_path, "r", encoding="utf-8") as f:
        return f.read()

def generate_questions_from_markdown(content, num_pairs=3):
    """
    Use OpenAI to generate question-answer pairs based on markdown content.
    """
    prompt = f"""You are helping a student understand their assignment.
Given the following documentation content from a markdown file:

\"\"\"
{content}
\"\"\"

Generate {num_pairs} questions a student might ask when trying to complete this assignment, along with clear and accurate answers based on the content above. Format your response as:

Q: <question>
A: <answer>

Repeat for each pair.
"""
    response = client.chat.completions.create(
        model="gpt-3.5-turbo",  # or "gpt-4" if available
        messages=[{"role": "user", "content": prompt}],
        temperature=0.7
    )
    return response.choices[0].message.content

if __name__ == "__main__":
    base_dir = "/Users/terriannezhang/Desktop/tai/tai/rag/tests/cs61a_200/course_website/lab"  # Change this to your actual root directory
    md_paths = find_all_markdown_files(base_dir)

    print(md_paths)

    if not md_paths:
        print("No markdown files found.")
    else:
        all_questions = []

        for md_path in md_paths:
            print(f"Markdown file found: {md_path}")
            content = read_markdown_file(md_path)

            chunks = split_into_chunks(content)

            for chunk in chunks:
                questions = generate_questions_from_markdown(chunk)
                all_questions.extend(q.strip() for q in questions.split("\n") if q.strip())

        with open("cs61a_questions.csv", mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["Question", "Answer"])
            for qa in all_questions:
                if qa.startswith("Q:"):
                    question = qa[3:].strip()
                elif qa.startswith("A:"):
                    answer = qa[3:].strip()
                    writer.writerow([question, answer])

        print("All questions written to cs61a_questions.csv")
