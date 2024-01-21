from scrape_class import RSTParser
import requests
import json
import os

def fetch_and_save_data(filename, url):
    """
    Fetch data from a given URL, save it to a file, and print certain parts of the JSON.

    :param url: The URL to fetch data from.
    :return: None
    """
    print(f"url:{url}")
    response = requests.get(url)

    # Error handling for HTTP request
    if response.status_code != 200:
        print("Failed to retrieve the content.")
        exit()
        return

    try:
        data = json.loads(response.text)
    except json.JSONDecodeError:
        print("Failed to parse the response as JSON.")
        return

    # Saving the entire fetched JSON to a file
    data=data['payload']['blob']['rawLines']
    filename = filename.split("/")[-1]
    with open(f"{filename}.rst", "w", encoding='utf-8') as outfile:
        for i in data:
            outfile.write(i)
            outfile.write("\n")
    print("Saved the fetched data to a file.")
    folder_name = filename

    # Check if folder exists. If not, create it.
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
        print(f"'{folder_name}' has been created.")
    else:
        print(f"'{folder_name}' already exists.")


def tree_call(cur, prev):
    # Test the function
    if cur=='*':
        return
    print(f"Fetching data from {cur}.rst")
    print(f"prev:{prev}")
    print(f"cur:{cur}")
    url = f"https://github.com/ros2/ros2_documentation/blob/galactic/source/{prev}{cur}.rst?plain=1"
    fetch_and_save_data(cur,url)
    filename = f'{cur}.rst'  # Replace with the name of your .rst file
    parser = RSTParser(filename)
    parser.print_header_tree()
    # parser.save_parsed_data()
    # parser.concat_print()
    print("Toctree Content:")
    print(parser.toctree_content)
    current_directory = os.getcwd()
    print(f"Current directory is {current_directory}")
    
    for link in parser.toctree_content:
        link = link.split("/")
        
        cur = cur.split("/")[-1]
        os.chdir(f"{current_directory}/{cur}")
        if len(link)==1:
            last=link[-1]
            last = last.replace('.rst', '') if last.endswith('.rst') else last
            tree_call(last, f"{prev}")
        else:   
            def func(s):
                parts = s
                if len(parts) <= 1:  # only one word
                    return ''
                return '/'.join(parts[1:])  # return everything after the first word
            last=func(link)
            last = last.replace('.rst', '') if last.endswith('.rst') else last
            tree_call(last, f"{prev}{cur}/")
        os.chdir(current_directory)
    

if __name__ == "__main__":
    starting='index'
    tree_call(starting,"")
