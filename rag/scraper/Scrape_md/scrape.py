import requests
import base64
import yaml
import os
import json
from header import MarkdownParser


def main():
    """
    The main function that initializes and orchestrates the process of scraping GitHub repositories' mkdocs.yml files.
    It sets up the URL of the mkdocs.yml file from various repositories and initiates the scraping process.
    """
    # TODO
    # Carla
    # create_and_enter_dir('carla')
    # url = "https://github.com/carla-simulator/carla/blob/master/mkdocs.yml"

    # Mkdocs
    # create_and_enter_dir('mkdocs')
    # url = "https://github.com/mkdocs/mkdocs/blob/master/mkdocs.yml"
    # uwasystemhealth
    # create_and_enter_dir('uwasystemhealth')
    # url = "https://github.com/uwasystemhealth/shl-mkdocs-tutorial-and-template/blob/template/mkdocs.yml"
    # MonashDataFluency
    create_and_enter_dir('MonashDataFluency')
    url = "https://github.com/MonashDataFluency/python-web-scraping/blob/master/mkdocs.yml"
    # MkDocsMaterial
    # create_and_enter_dir('mkdocs')
    # url = "https://github.com/squidfunk/mkdocs-material/blob/master/mkdocs.yml"
    # openml
    # create_and_enter_dir('openml')
    # url = "https://github.com/openml/docs/blob/master/mkdocs.yml"
    # awsome-kurbenetes
    # create_and_enter_dir('awsome-kurbenetes')
    # url = "https://github.com/nubenetes/awesome-kubernetes/blob/master/mkdocs.yml"
    # python-web-scraping
    # create_and_enter_dir('python-web-scraping')
    # url = "https://github.com/MonashDataFluency/python-web-scraping/blob/master/mkdocs.yml"

    headers = {'Accept': 'application/json'}
    response = requests.get(url, headers=headers)
    data = response.json()
    print(data)
    content = data['payload']['blob']['rawLines']
    content = '\n'.join(content)
    content = extract_yaml_sections(content)
    parsed_yaml = yaml.load(content, Loader=yaml.SafeLoader)
    repo_url = parsed_yaml['repo_url']
    edit_url = parsed_yaml.get('edit_uri')
    if edit_url:
        edit_url.replace('\\', '/').replace('edit/', 'blob/')
    docs_dir = parsed_yaml.get('docs_dir', None)
    if docs_dir:
        base_url = os.path.join(cd_back_link(url), docs_dir)
    else:
        base_url = os.path.join(cd_back_link(url), "docs/")
    base_url = replace_backslash_with_slash(base_url)
    nav = parsed_yaml['nav']
    fetch_urls(base_url, nav)


def cd_back_link(url, num_parts_to_remove=1):
    """
    Navigates back in the URL structure by the specified number of parts.
    - url (str): The URL to navigate back from.
    - num_parts_to_remove (int, optional): The number of segments to remove from the end of the URL. Default is 1.
    - Returns: The modified URL.
    """
    if not url:
        return ""
    for _ in range(num_parts_to_remove):
        url = url.rsplit('/', 1)[0]
        return url
def create_and_enter_dir(directory_name):
    """
    Creates a directory with the given name and changes the current working directory to it.
    - directory_name (str): The name of the directory to be created.
    """
    # Create the directory if it doesn't exist
    if not os.path.exists(directory_name):
        os.mkdir(directory_name)
    
    # Change the current working directory
    os.chdir(directory_name)

def get_save_content(file_name, url):
    """
    Retrieves content from the given URL and saves it to a file.
    - file_name (str): Name of the file to save the content.
    - url (str): URL to fetch the content from.
    - Returns: The content fetched from the URL.
    """
    # Fetch the content from the URL
    headers = {'Accept': 'application/json'}
    response = requests.get(url, headers=headers)
    data = response.json()
    content = data['payload']['blob']['rawLines']
    
    # Convert the content list into a single string
    content = '\n'.join(content)
    
    # Save the content into the specified file
    with open(file_name, 'w', encoding='utf-8') as file:
        file.write(content)
    
    return content

def fetch_file_content_from_github(url):
    """
    Fetches the file content from a GitHub URL.
    - url (str): The GitHub URL from which the file content is to be fetched.
    - Returns: The decoded content of the file.
    """
    response = requests.get(url)
    data = response.json()
    content = base64.b64decode(data['content']).decode('utf-8')
    return content

def get_url_child(url):
    """
    Fetches child markdown file names from a given GitHub directory URL.
    - url (str): The GitHub directory URL to fetch the markdown files from.
    - Returns: A list of markdown file names.
    """
    headers = {'Accept': 'application/json'}
    response = requests.get(url, headers=headers)
    data = response.json()
    childs=[]
    md_files=data["payload"]["tree"]["items"]
    for i in md_files:
        name = i['name']
        if name.endswith('.md'):
            childs.append(name)
    return childs

def fetch_urls(base_url, nav):
    """
    Recursively fetches URLs and their content based on the navigation structure defined in the mkdocs.yml file.
    - base_url (str): The base URL for the mkdocs documentation.
    - nav (list): A list or dictionary defining the navigation structure from mkdocs.yml.
    """
    print(f"nav: {nav}")
    # nav = [list(i.values())[0] if isinstance(i, dict) else i for i in nav]
    for i in nav:
        cur_dir=os.getcwd()
        if isinstance(i, str) and i.endswith(".md"):
            filename=i.split("/")[-1]
            url=os.path.join(base_url,i)
            url+="?plain=1"
            parser=MarkdownParser(url, filename.replace('.md',''))
            if parser.fail:
                continue
            parser.print_header_tree()
            parser.print_segment()
            parser.concat_print()
            
            get_save_content(filename,url)
            os.chdir(cur_dir)
            continue
        key, value = i.popitem()
        if(not key):
            continue
        create_and_enter_dir(key)
        # if it is nested
        if isinstance(value, list):
            fetch_urls(base_url,value)
        # if it is not md
        elif value.endswith('/') and not value.startswith('http'):
            cur_child_dir=os.getcwd()
            url=os.path.join(base_url,value)
            childs=get_url_child(url)
            print(childs)
            for child in childs:
                filename=value.split("/")[-1]
                create_and_enter_dir(child.replace('.md',''))
                child_url=os.path.join(url,child)
                child_url+="?plain=1"
                filename=child
                parser=MarkdownParser(child_url, filename.replace('.md',''))
                if parser.fail:
                    continue
                get_save_content(filename,child_url)
                parser.print_header_tree()
                parser.print_segment()
                parser.concat_print()
                os.chdir(cur_child_dir)
                
        elif not value.endswith('.md'):
            continue
        else:
            filename=value.split("/")[-1]
            url=os.path.join(base_url,value)
            url+="?plain=1"
            parser=MarkdownParser(url, filename.replace('.md',''))
            if parser.fail:
                continue
            parser.print_header_tree()
            parser.print_segment()
            parser.concat_print()
            filename=value.split("/")[-1]
            get_save_content(filename,url)

        os.chdir(cur_dir)
def replace_backslash_with_slash(path):
    """
    Replaces backslashes with slashes in a given file path.
    - path (str): The file path to modify.
    - Returns: The modified file path with slashes.
    """
    return path.replace('\\', '/')

def create_and_enter_dir(directory_name):
    """
    Creates a directory with the specified name and changes the current working directory to it.
    - directory_name (str): The name of the directory to be created.
    """
    # Create the directory if it doesn't exist
    directory_name = directory_name.replace('/', '-')
    if not os.path.exists(directory_name):
        os.mkdir(directory_name)
    
    # Change the current working directory
    os.chdir(directory_name)

def extract_yaml_sections(data: str) -> str:
    """
    Extract specific sections from the provided YAML data string.
    
    :param data: A string containing the YAML data.
    :return: A string containing the extracted sections.
    """
    lines = data.split("\n")

    result = []
    for i in range(len(lines)):
        stripped_line = lines[i].strip()
        # Check if the line starts with one of our desired keys
        if stripped_line.startswith(("repo_url:", "edit_uri:","docs_dir:")):
            result.append(lines[i])
        elif stripped_line.startswith("nav:"):
            result.append(lines[i])
            cur=i+1
            while lines[cur].strip().startswith("-") or lines[cur].strip() == ("") or lines[cur].strip().startswith("#"):
                result.append(lines[cur])
                cur+=1
                if cur>=len(lines): 
                    break
            break
        # If in capture mode, add the line to the result


    # Combine the result lines back into a string
    output = "\n".join(result)

    return output



if __name__ == '__main__':
    main()
    
