import json
import csv

file_path = '/Users/terriannezhang/Desktop/tai/tai/rag/tests/EECS_C106B_206B.json'
with open(file_path, 'r') as f:
    data = json.load(f)

question_posts_by_students = []
admin_posts = []
qa_pairs = []

for post in data:
    # Check if the post is a question and made by a student
    if post['type'] == 'question' and post['user']['role'] == 'student':
        question_posts_by_students.append(post)

    # Check if the post is a post made by an admin
    if post['type'] == 'post' and post['user']['role'] == 'admin':
        admin_posts.append(post)

for post in question_posts_by_students:
    question = post.get('text', '')
    answer = None
    category = post.get('title', '')

    # Look for the first answer by an admin
    if 'answers' in post:
        for answer_post in post['answers']:
            if answer_post['user']['role'] == 'admin':
                answer = answer_post.get('text', '')
                break
    
    if question and answer:
        qa_pairs.append([question, answer, category])

for post in admin_posts:
    category = post.get('title', '')

    for comment in post.get('comments', []):
        if comment['user']['role'] == 'student':
            question = comment.get('text', '')
            answer = None

            # Look for the first reply to this comment by an admin
            for reply in comment.get('comments', []):
                if reply['user']['role'] == 'admin':
                    answer = reply.get('text', '')
                    break
            
            if question and answer:
                qa_pairs.append([question, answer, category])

# Write the question-answer pairs to a CSV file
output_file = '/Users/terriannezhang/Desktop/tai/tai/rag/tests/qa_pairs.csv'

with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Question', 'Answer', 'Category'])
    writer.writerows(qa_pairs)

print(f"QA pairs successfully written to {output_file}")