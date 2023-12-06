from langchain.document_loaders import UnstructuredHTMLLoader

import os
import requests
from bs4 import BeautifulSoup

# Fetch the webpage content
url = "http://wiki.ros.org/ROS/Tutorials"  # Replace with your target URL
response = requests.get(url)

# Ensure the request was successful
response.raise_for_status()

# Use BeautifulSoup to parse the HTML
soup = BeautifulSoup(response.content, 'html.parser')

# Store the prettified HTML
html_content = soup.prettify()

# Create 'html' directory if it doesn't exist
if not os.path.exists('html'):
    os.makedirs('html')

# Define a filename. You can customize this as per your requirements.
filename = os.path.join('html', 'output.html')

# Save the content to the file
with open(filename, 'w', encoding='utf-8') as file:
    file.write(html_content)

print(f"HTML content saved to {filename}")



loader = UnstructuredHTMLLoader("html/output.html")
data = loader.load()
for i in data:
    print(i)
    print("=====================================")