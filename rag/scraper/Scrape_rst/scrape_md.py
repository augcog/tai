from scrape_class import RSTParser

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'Scrape_header')))

from header import MarkdownParser
import requests
import json
import os
import re
from termcolor import colored
from rst_to_myst import rst_to_myst
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
    mkdir('Sawyer_md')
    os.chdir('Sawyer_md')
    starting = 'index'
    url = f"https://github.com/ros-planning/moveit_tutorials/blob/master/index.rst?plain=1"

    # conda getting started
    # mkdir('conda_getting_started_md')
    # os.chdir('conda_getting_started_md')
    # starting='index'
    # url = "https://github.com/conda/conda/blob/main/docs/source/user-guide/index.rst?plain=1"

    home_url = url.rsplit('/', 1)[0]
    # Current directory
    home_dir = os.getcwd()
    tree_call(starting, url, home_url, home_dir)


def fetch_and_save_data(filename, url):
    """
    Fetch data from a given URL, save it to a file, and print certain parts of the JSON.

    :param url: The URL to fetch data from.
    :return: None
    """
    print(f"url:{url}")
    headers = {'Accept': 'application/json'}
    response = requests.get(url, headers=headers)
    data = response.json()

    # Saving the entire fetched JSON to a file
    data='\n'.join(data['payload']['blob']['rawLines'])
    # print(data)
    filename = filename.split("/")[-1]
    with open(f"{filename}.rst", "w", encoding='utf-8') as outfile:
        outfile.write(data)
    with open(f"{filename}.md", "w", encoding='utf-8') as outfile:
        md_data=rst_to_myst(data).text
        outfile.write(md_data)
    print("Saved the fetched data to a file.")

def mkdir(dir_name):
    # Normalize the directory path to handle cases like "dir1//dir2"
    dir_name = os.path.normpath(dir_name)

    if not os.path.exists(dir_name):
        os.makedirs(dir_name)
        print(f"{dir_name} created!")
    else:
        print(f"{dir_name} already exists!")

def tree_call(cur_file, url, home_url, home_dir):
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
        # To verify, let's print the current working directory after the change(s)
        # print(os.getcwd())
    parser = RSTParser(os.path.join(os.getcwd(),f"{filename}.rst"))
    parser_md = MarkdownParser(os.path.join(os.getcwd(),f"{filename}.md"))
    parser_md.print_header_tree()
    parser_md.print_segment()
    parser_md.concat_print()
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