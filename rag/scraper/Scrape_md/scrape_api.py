import requests
import base64
import yaml
import os

def github_url_to_api_url(github_url):
    # Normalize the URL to use forward slashes and replace the word 'edit' with 'blob'
    github_url = github_url.replace('\\', '/').replace('/edit/', '/blob/')
    
    # Split the URL and extract the relevant parts
    parts = github_url.split('/')
    owner = parts[3]
    repo = parts[4]
    branch = parts[6]
    file_path = '/'.join(parts[7:])

    # Construct the API URL
    api_url = f"https://api.github.com/repos/{owner}/{repo}/contents/{file_path}"
    
    return api_url


def fetch_file_content_from_github(url):
    response = requests.get(url)
    data = response.json()
    print(data)
    content = base64.b64decode(data['content']).decode('utf-8')
    return content

def fetch_urls(base_url, nav):
    for i in nav:
        key, value = i.popitem()
        if isinstance(value, list):
            fetch_urls(base_url,value)
        else:
            url=os.path.join(base_url,value)
            api_url=github_url_to_api_url(url)
            fetch_file_content_from_github(api_url)
            print(api_url)
            


if __name__ == '__main__':
    url="https://github.com/carla-simulator/carla/blob/master/mkdocs.yml"
    api_url=github_url_to_api_url(url)
    print(api_url)
    api_url = 'https://api.github.com/repos/mkdocs/mkdocs/contents/mkdocs.yml'
    
    content = fetch_file_content_from_github(api_url)

    # Parse the YAML content
    parsed_yaml = yaml.safe_load(content)
    # print(parsed_yaml)
    # Extract specific information from the YAML data
    # site_url = parsed_yaml['site_url']
    repo_url = parsed_yaml['repo_url']
    edit_url = parsed_yaml['edit_uri']
    print(f"repo:{repo_url}")
    print(f"edit:{edit_url}")
    base_url = os.path.join(repo_url,edit_url)
    print("base:"+base_url)
    nav = parsed_yaml['nav']
    fetch_urls(base_url, nav)

    # Print the extracted information
    
