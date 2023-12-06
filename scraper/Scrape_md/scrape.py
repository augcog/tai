import requests
import base64
import yaml
import os
import json
from header import MarkdownParser

def create_and_enter_dir(directory_name):
    # Create the directory if it doesn't exist
    if not os.path.exists(directory_name):
        os.mkdir(directory_name)
    
    # Change the current working directory
    os.chdir(directory_name)

def get_save_content(file_name, url):
    # Fetch the content from the URL
    response = requests.get(url)
    data = json.loads(response.text)
    content = data['payload']['blob']['rawLines']
    
    # Convert the content list into a single string
    content = '\n'.join(content)
    
    # Save the content into the specified file
    with open(file_name, 'w', encoding='utf-8') as file:
        file.write(content)
    
    return content

def fetch_file_content_from_github(url):
    response = requests.get(url)
    data = response.json()
    print(data)
    content = base64.b64decode(data['content']).decode('utf-8')
    return content

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

def fetch_urls(base_url, nav):
    for i in nav:

        cur_dir=os.getcwd()
        print(i)
        if isinstance(i, str) and i.endswith(".md"):
            filename=i.split("/")[-1]
            url=os.path.join(base_url,i)
            url+="?plain=1"
            print(url)
            parser=MarkdownParser(url, filename)
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
            print(url)
            childs=get_url_child(url)
            for child in childs:
                filename=i.split("/")[-1]
                create_and_enter_dir(child.replace('.md',''))
                child_url=os.path.join(url,child)
                child_url+="?plain=1"
                print(child_url)
                filename=child
                parser=MarkdownParser(child_url, filename)
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
            print(url)
            parser=MarkdownParser(url, filename)
            if parser.fail:
                continue
            parser.print_header_tree()
            parser.print_segment()
            parser.concat_print()
            filename=value.split("/")[-1]
            get_save_content(filename,url)

        os.chdir(cur_dir)
def replace_backslash_with_slash(path):
    return path.replace('\\', '/')

def create_and_enter_dir(directory_name):
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
    # Carla
    url="https://github.com/carla-simulator/carla/blob/master/mkdocs.yml"
    # Mkdocs
    # url = "https://github.com/mkdocs/mkdocs/blob/master/mkdocs.yml"
    # uwasystemhealth
    # url = "https://github.com/uwasystemhealth/shl-mkdocs-tutorial-and-template/blob/template/mkdocs.yml"
    # MonashDataFluency
    # url = "https://github.com/MonashDataFluency/python-web-scraping/blob/master/mkdocs.yml"
    # MkDocsMaterial
    # url = "https://github.com/squidfunk/mkdocs-material/blob/master/mkdocs.yml"
    # openml
    # url = "https://github.com/openml/docs/blob/master/mkdocs.yml"
    # awsome-kurbenetes
    # url = "https://github.com/nubenetes/awesome-kubernetes/blob/master/mkdocs.yml"
    # python-web-scraping
    # url = "https://github.com/MonashDataFluency/python-web-scraping/blob/master/mkdocs.yml"
    def cd_back_link(url, num_parts_to_remove=1):
        if not url:
            return ""
        for _ in range(num_parts_to_remove):
            url = url.rsplit('/', 1)[0]
            return url

   
    
    response=requests.get(url)
    data=json.loads(response.text)  
    # print(json.dumps(data, indent=4))   
    content=data['payload']['blob']['rawLines']
    content = '\n'.join(content)
    print(extract_yaml_sections(content))
    content=extract_yaml_sections(content)
    # # print(content)

    parsed_yaml = yaml.load(content, Loader=yaml.SafeLoader)

    # print(json.dumps(parsed_yaml, indent=4))
    repo_url = parsed_yaml['repo_url']
    edit_url = parsed_yaml.get('edit_uri')
    if edit_url:
        edit_url.replace('\\', '/').replace('edit/', 'blob/')
    docs_dir = parsed_yaml.get('docs_dir', None)

    
    print(f"repo:{repo_url}")
    print(f"edit:{edit_url}")
    if docs_dir:
        base_url = os.path.join(cd_back_link(url),docs_dir)
    else:
        base_url = os.path.join(cd_back_link(url),"docs/")
    base_url = replace_backslash_with_slash(base_url)
    print("base:"+base_url)
    nav = parsed_yaml['nav']
    # Name
    create_and_enter_dir('carla')

    fetch_urls(base_url, nav)
    # get_content("https://github.com/carla-simulator/carla/blob/master/Docs/index.md?plain=1")


    # Print the extracted information
    
