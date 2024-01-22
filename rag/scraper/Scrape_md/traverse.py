import requests
import json
URL = "https://github.com/mkdocs/mkdocs/tree/master/docs/user-guide/"

def get_url_child(url):
    response = requests.get(url)
    childs=[]
    data=json.loads(response.text)
    md_files=data["payload"]["tree"]["items"]
    for i in md_files:
        name = i['name']
        if name.endswith('.md'):
            childs.append(name)
    return childs
