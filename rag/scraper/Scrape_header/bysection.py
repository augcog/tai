import requests
from bs4 import BeautifulSoup

# Step 1: Fetch the Webpage Content
url = "https://numpy.org/install/"
response = requests.get(url)
html_content = response.text

# Step 2: Parse with BeautifulSoup
soup = BeautifulSoup(html_content, 'html.parser')

# Step 3: Segment the Content
# This is a hypothetical segmenting operation, as I can't view the real structure of the webpage.
# You'll have to inspect the page source to determine the right tags or classes to segment by.
sections = soup.find_all('div', class_='section')  # this is a guess!

for idx, section in enumerate(sections, 1):
    # Print the content of each section (adjust as per the actual structure of the page)
    print(f"Section {idx}:")
    print(section.prettify())
    print("\n")
