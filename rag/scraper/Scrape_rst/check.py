import requests
import json

url = "https://github.com/carla-simulator/carla/blob/master/Docs/index.md?plain=1"
response = requests.get(url)


data = json.loads(response.text)
print(data)
data=data['payload']['blob']['rawLines']
for i in data:
    print(i)
with open('json.txt', 'w') as outfile:
    json.dump(data, outfile, indent=4)
print(json.dumps(data, indent=4))