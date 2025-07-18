from pathlib import Path

from rag.file_conversion_router.classes.new_page import Page
index_helper = {'Lecture 1: Introduction to Python Programming': 1, 'Lecture 1: Introduction to Python Programming>Exercises': 13, 'Lecture 1: Introduction to Python Programming>Exercises>Exercise 1': 14, 'Lecture 1: Introduction to Python Programming>Exercises>Exercise 2': 16, 'Lecture 1: Introduction to Python Programming>Exercises>Exercise 3': 18, 'Lecture 1: Introduction to Python Programming>Exercises>Exercise 4': 20, 'Lecture 1: Introduction to Python Programming>Exercises>Exercise 5': 22, 'Lecture 1: Introduction to Python Programming>Keywords': 2, 'Lecture 1: Introduction to Python Programming>Running First Python Code in Command Line': 3, 'Lecture 1: Introduction to Python Programming>Summary': 12}
file_type = 'ipynb'
course_name = 'roar'
stem = '1-1-introduction-to-python-programming'
url = ''
metadata_path = Path('/Users/yyk956614/tai/rag/test_folder/1-1-introduction-to-python-programming_metadata.yaml')
md_path = Path('/Users/yyk956614/tai/rag/test_folder_output/1-1-introduction-to-python-programming/1-1-introduction-to-python-programming.md')
with open(md_path, 'r', encoding='utf-8') as f:
    content = f.read()

content = {'text':content}
page = Page(filetype= file_type, index_helper=index_helper, page_url=url,content=content)
page.to_chunk()
page.chunks_to_pkl(output_path='/Users/yyk956614/tai/rag/test_folder_output/1-1-introduction-to-python-programming/1-1-introduction-to-python-programming1.pkl')
