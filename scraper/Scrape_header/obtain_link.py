import re

# Read the markdown file
with open('markdown.md', 'r') as file:
    content = file.read()

# Regular expression pattern to match markdown links
# Capture the URL but ignore any optional title in double quotes afterwards
pattern = r'\[.*?\]\(([^" ]+)(?: ".*?")?\)'

# Find all matches
links = re.findall(pattern, content)

# Print the links
for link in links:
    print(link)
