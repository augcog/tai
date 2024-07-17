import json
from filter import json_kb_filter
# from rag.file_conversion_router.api import convert_directory

def main():
    # Load JSON data from test.json
    with open('106b_ed.json', 'r') as file:
        json_data = json.load(file)
    # Convert JSON data to Markdown
    markdown_content_qa = convert_json_to_markdown_qa(json_data)
    markdown_content = convert_json_to_markdown(json_data)

    # Save Markdown content to output.md
    save_markdown(markdown_content_qa, 'input_mds/outputqa.md')
    save_markdown(markdown_content, 'input_mds/outputkb.md')

    # convert the markdowns to pdf/md/pkl formats
    # convert_directory('./input_mds/outputqa.md', './output_mds/outputqa')
    # convert_directory('./input_mds/outputkb.md', './outpud_mds/outputkb')

def process_comments(comments):
    markdown = ""
    for comment in comments:
        markdown += f"**User:** {comment['user']['name']}\n"
        markdown += f"**Role:** {comment['user']['role']}\n"
        markdown += f"**URL:** {comment['url']}\n"
        markdown += f"{comment['text']}\n"
        if "comments" in comment and len(comment.get("comments", [])) > 0:
            markdown += process_comments(comment["comments"])
            markdown += '\n'
    return markdown

def process_comments_qa(comments):
    markdown = ""
    for comment in comments:
        markdown += f"**Role:** {comment['user']['role']}\n"
        markdown += f"**URL:** {comment['url']}\n"
        markdown += f"{comment['text']}\n"
        if "comments" in comment and len(comment.get("comments", [])) > 0:
            markdown += process_comments(comment["comments"])
            markdown += '\n'
    return markdown

def convert_json_to_markdown_qa(data):
    markdown = ""
    for item in data:
        if item["type"] in ["post", "announcement"]:
            markdown += f"# {item['title']}\n"
            for comment in item.get("comments", []):
                markdown += "## Question\n"
                markdown += f"**Role:** {comment['user']['role']}\n"
                markdown += f"**URL:** {comment['url']}\n"
                markdown += f"{comment['text']}\n"
                markdown += "## Response\n"
                if "comments" in comment and len(comment.get("comments", [])) > 0:
                    markdown += process_comments_qa(comment["comments"])
        elif item["type"] == "question":
            markdown += f"# {item['title']}\n"
            markdown += "## Question\n"
            markdown += f"**Role:** {item['user']['role']}\n"
            markdown += f"**URL:** {item['url']}\n"
            markdown += f"{item['text']}\n"
            markdown += "## Response\n"
            for answer in item.get("answers", []):
                markdown += f"**Role:** {answer['user']['role']}\n"
                markdown += f"**URL:** {answer['url']}\n"
                markdown += f"{answer['text']}\n"
    return markdown

def convert_json_to_markdown(data):
    # first filter the data!
    data = json_kb_filter(data)

    markdown = ""
    for item in data:
        markdown += f"# {item['title']}\n"
        markdown += f"**User:** {item['user']['name']}\n"
        markdown += f"**Role:** {item['user']['role']}\n"
        markdown += f"**URL:** {item['url']}\n"
        markdown += f" {item['text']}\n"

        if len(item.get("comments", [])) > 0:
            for comment in item.get("comments", []):
                markdown += "### Comment\n"
                markdown += f"**User:** {comment['user']['name']}\n"
                markdown += f"**Role:** {comment['user']['role']}\n"
                markdown += f"**URL:** {comment['url']}\n"
                markdown += f"{comment['text']}\n"
                if "comments" in comment and len(comment.get("comments", [])) > 0:
                    markdown += process_comments(comment["comments"])
        if len(item.get("answers", [])) > 0:
            for answer in item.get("answers", []):
                markdown += "### Answer \n"
                markdown += f"**Name:** {answer['user']['name']}\n"
                markdown += f"**Role:** {answer['user']['role']}\n"
                markdown += f"**URL:** {answer['url']}\n"
                markdown += f"{answer['text']}\n"
                if "comments" in answer and len(answer.get("comments", [])) > 0:
                    markdown += process_comments(answer["comments"])

    return markdown

def save_markdown(markdown_content, file_path):
    with open(file_path, 'w') as file:
        file.write(markdown_content)

if __name__ == "__main__":
    main()
