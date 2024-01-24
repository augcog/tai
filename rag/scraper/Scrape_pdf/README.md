# Scrape_pdf  
First we will need to convert the pdf into a markdown format. We will use a tool called nougat.
- run `pip install nougat-ocr` to install nougat
- Go to `nougat.py` and choose the pdf you want to convert and the name of the folder you want to save your documents at.
  ```
  pdf_to_md('~/Downloads/MLS.pdf', 'textbook')
  ```
- After you get your markdown folder now run `header.py` to segment the contents of the markdown file into headers and contents.
  ```
  # TODO
  parser = MarkdownParser('textbook/MLS.mmd')
  ```
- After you have set up the variables you can run `python3 scrape.py` and it will start scraping the website.