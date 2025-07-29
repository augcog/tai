# from html_to_markdown import convert_to_markdown
# with open("/home/bot/bot/yk/YK_final/test_folder/cs61a/1.2 Elements of Programming.html", "r", encoding="utf-8") as file:
#     html = file.read()
# result = convert_to_markdown(html)
# print(result)

fp="/home/bot/bot/yk/YK_final/test_folder/cs61a/1.2 Elements of Programming.html"

import html
import chardet

# Detect encoding
with open(fp, 'rb') as file:
    raw_data = file.read()
    encoding_info = chardet.detect(raw_data)
    detected_encoding = encoding_info['encoding']

print(f"Detected encoding: {detected_encoding}")

# Open with detected encoding
with open(fp, 'r', encoding=detected_encoding) as file:
    content = file.read()

decoded_content = html.unescape(content)
print(decoded_content[:500])  # Print the first 500 characters to verify content
