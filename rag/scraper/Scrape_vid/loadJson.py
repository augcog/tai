import json

# Assuming the JSON content is stored in 'test.json'
# with open('transcribe_manifest.json', 'r', encoding='utf-8') as file:
#     data = json.load(file)
# # Print out the content to inspect it
# # print(content)
# print(file)
data = []
f = open('transcribe_manifest.json', 'r', encoding='utf-8')
f = json.load(f.read())
# Try parsing the content
try:
    # data = json.loads(content)
    # print("Parsed JSON:", data)
    for line in f.items():
        # print("this" + line)
        # item = json.loads(line)
        item = line
        data.append(item)
        print(item)
except json.decoder.JSONDecodeError as e:
    print("JSON decoding error:", e)

print(data)