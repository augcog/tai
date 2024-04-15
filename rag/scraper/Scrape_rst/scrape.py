from scrape_class import RSTParser
import requests
import json
import os
import re
from termcolor import colored
global parser
ignore=["glossary","*"]


def main():
    """
    The main function to start the scraping process.
    It sets up the initial directory, URL, and other parameters, and then calls tree_call to begin recursive scraping.
    """
    # Numpy
    # mkdir('numpy')
    # os.chdir('numpy')
    # starting='index'
    # url=f"https://github.com/numpy/numpy/blob/main/doc/source/index.rst?plain=1"

    # Sawyer
    mkdir('Sawyer')
    os.chdir('Sawyer')
    starting='index'
    url = f"https://github.com/ros-planning/moveit_tutorials/blob/master/index.rst?plain=1"

    # conda
    # mkdir('conda')
    # os.chdir('conda')
    # starting='index'
    # url = "https://github.com/conda/conda-docs/blob/main/docs/source/index.rst?plain=1"

    # conda getting started
    # mkdir('conda_getting started')
    # os.chdir('conda_getting started')
    # starting='index'
    # url = "https://github.com/conda/conda/blob/main/docs/source/user-guide/index.rst?plain=1"
    home_url= url.rsplit('/', 1)[0]
    # Current directory
    home_dir=os.getcwd()
    tree_call(starting,url,home_url,home_dir)


def fetch_and_save_data(filename, url):
    """
    Fetch data from a given URL, save it to a file, and print certain parts of the JSON.

    :param url: The URL to fetch data from.
    :return: None
    """
    headers = {'Accept': 'application/json'}
    response = requests.get(url, headers=headers)
    data = response.json()

    # Saving the entire fetched JSON to a file
    data=data['payload']['blob']['rawLines']
    filename = filename.split("/")[-1]
    with open(f"{filename}.rst", "w", encoding='utf-8') as outfile:
        for i in data:
            outfile.write(i)
            outfile.write("\n")
    print("Saved the fetched data to a file.")

def mkdir(dir_name):
    """
    Creates a directory with the given name if it doesn't already exist.
    - dir_name (str): The name of the directory to be created.
    """
    # Normalize the directory path to handle cases like "dir1//dir2"
    dir_name = os.path.normpath(dir_name)

    if not os.path.exists(dir_name):
        os.makedirs(dir_name)
        print(f"{dir_name} created!")
    else:
        print(f"{dir_name} already exists!")

def tree_call(cur_file, url, home_url, home_dir):
    """
    Recursively navigates through the file structure of a website and fetches content based on RST files.
    - cur_file (str): The current file being processed.
    - url (str): The URL of the current file.
    - home_url (str): The base URL of the website.
    - home_dir (str): The base directory on the local filesystem.
    """
    filename=f"{cur_file}"
    code=fetch_and_save_data(filename,url)
    if code==1:
        return
    def cd_back_link(url, num_parts_to_remove=1):
        if not url:
            return ""

        for _ in range(num_parts_to_remove):
            url = url.rsplit('/', 1)[0]

        return url + '/'
    def cd_dir(num=1):
        for _ in range(num):
            os.chdir('..')
    parser = RSTParser(f"{filename}.rst")
    parser.print_header_tree()
    parser.save_parsed_data()
    parser.concat_print()
    url=cd_back_link(url)
    current_directory = os.getcwd()
    
    for sublink in parser.toctree_content:
        if sublink.startswith('/'):
            sublink=sublink[1:]
            part=sublink.split("/")
            cur_name,dir=part[-1],'/'.join(part[:-1])
            if cur_name=='*' or re.match(r'^https.*',sublink) is not None:
                continue
            os.chdir(home_dir)
            mkdir(dir)
            if dir:
                os.chdir(dir)
            sublink=sublink[:-4] if sublink.endswith('.rst') else sublink
            temp_url=url
            url=home_url+sublink+".rst?plain=1"
            print(url)
            tree_call(cur_name, url, home_url, home_dir)
            url=temp_url
            os.chdir(current_directory)
            
        else:
            part=sublink.split("/")
            cur_name,dir=part[-1],'/'.join(part[:-1])
            if cur_name in ignore or re.match(r'^https.*',sublink) is not None:
                continue
            mkdir(dir)
            if dir:
                os.chdir(dir)
            sublink=sublink[:-4] if sublink.endswith('.rst') else sublink
            url+=sublink+".rst?plain=1"
            print(url)
            tree_call(cur_name, url, home_url, home_dir)
            url=cd_back_link(url, len(part))
            os.chdir(current_directory)



if __name__ == "__main__":
    main()
