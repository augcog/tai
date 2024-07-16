# Scrape_pdf  
First we will need to convert the pdf into a markdown format. We will use two tools called nougat and pix2text.
- run `pip install nougat-ocr` to install nougat
- run `pip install pix2tex` to install pix2text
- Go to `Scrape_pdf.py` and choose the pdf you want to convert and the name of the folder you want to save your documents at.
- change the path in Scrape_pdf.py to your file path and run  

  ```
- After you get your markdown folder now run `header.py` to segment the contents of the markdown file into headers and contents.
  ```
  # TODO
  ```
- After you have set up the variables you can run `python3 scrape.py` and it will start scraping the website.
nougat will speed up at computer with gpu