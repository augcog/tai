import requests
from bs4 import BeautifulSoup, NavigableString

class TreeNode:
    def __init__(self, tag, content=""):
        self.tag = tag
        self.content = content.strip()
        self.children = []

    def add_child(self, node):
        self.children.append(node)

def build_tree(tag):
    """
    Recursively build the tree from the HTML, only creating nodes for specified tags.
    """
    if isinstance(tag, NavigableString):
        content = tag.strip()
        if content:  # If the string is not just whitespace and within a desired tag
            return TreeNode(tag=None, content=content)
        return None  # Important to return None if the string is just whitespace.

    # Check if the current tag is one of the desired tags
    if tag.name in ['title', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6']:
        node = TreeNode(tag=tag.name)
        for child in tag.children:
            child_node = build_tree(child)
            if child_node:
                node.add_child(child_node)
        return node

    # If the current tag is not one of the desired tags, still traverse its children
    # but don't create a node for it.
    valid_child_results = []
    for child in tag.children:
        child_result = build_tree(child)  # Capture any result from the child tags
        if child_result:  # If a child returned a result, append it to the list.
            if isinstance(child_result, TreeNode):
                valid_child_results.append(child_result)
            else:  # If the child_result is a list of TreeNodes
                valid_child_results.extend(child_result)

    # After traversing all children, return the list of valid child results.
    # If it's empty, return None.
    return valid_child_results if valid_child_results else None





def print_tree(node, indent=0):
    """
    Recursively print the tree structure.
    """
    if isinstance(node, list):
        for n in node:
            print_tree(n, indent)
        return

    if node.tag:
        print('  ' * indent + f"Tag: {node.tag}")
        if node.content:
            print('  ' * indent + f"Content: {node.content}")
    else:
        print('  ' * indent + f"Content: {node.content}")

    for child in node.children:
        print_tree(child, indent+1)


def main(url):
    response = requests.get(url)
    soup = BeautifulSoup(response.content, 'html.parser')
    root = build_tree(soup.html)

    print_tree(root)

if __name__ == '__main__':
    url = 'https://docs.ros.org/en/galactic/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html'  # Replace this with the URL of your choice
    main(url)
