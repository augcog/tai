import pypandoc
import requests
from bs4 import BeautifulSoup

# URL of the website
url = 'https://docs.python.org/3/library/random.html'

# Fetch the content of the website
response = requests.get(url)

# Parse the HTML content
soup = BeautifulSoup(response.text, 'html.parser').prettify()
from markdownify import markdownify as md

# output = pypandoc.convert_file('EE106A_Lab1_ROS.tex', 'md', format='latex')
output = pypandoc.convert_file('EE106A_Lab1_ROS.tex', 'md', format='latex', outputfile="EE106A_Lab1_ROS.md")
output = pypandoc.convert_text(soup, 'md', format='html', outputfile="soup.md")
print(output)