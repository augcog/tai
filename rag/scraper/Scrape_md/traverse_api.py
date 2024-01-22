import requests
import json
# handle cases like blablabla/
def fetch_and_process_data(url):
    response = requests.get(url)
    data = response.json()

    # Extract names from the JSON data
    names = [item['name'] for item in data]
    
    return names

if __name__ == '__main__':
    url = 'https://api.github.com/repos/mkdocs/mkdocs/contents/docs/user-guide/'
    names = fetch_and_process_data(url)
    for name in names:
        print(name)
