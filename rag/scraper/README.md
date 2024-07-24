# scraper

## Supported Formats
The following scrapers can be found under the [Scrape_master](Scraper_master/) folder:
- [scrape_header.py](Scraper_master/scrape_header.py): For general websites
- [scrape_md.py](Scraper_master/scrape_md.py): For websites that use markdown
- [scrape_rst.py](Scraper_master/scrape_rst.py): For websites that use rst
- [scrape_pdf.py](Scraper_master/scrape_pdf.py): For pdfs
- [scrape_vid.py](Scraper_master/scrape_vid.py): For videos

## Scraper and Embedding Pipeline
Use [pipline_kb.py](Scraper_master/pipeline_kb.py) as a pipeline to scrape, chunk and embed websites into a knowledge base. The pipeline first scrapes, and then converts the content into markdown. Finally, it embeds and saves the everything as a knowledge base. This is all saved according to the path defined by root_folder. The knowledge base is automatically saved in the scraped data folder in a folder labeled "pickle". 
    A .yaml file is used to specify the tasks to be performed. It should be should be structured as follows:
    root_folder : "path/to/root/folder"
    tasks :
      - name : "Website Name"
        url : "https://website.url"
      - name : "Website Name"
        url : "https://website.url"

## Scraper Structure and Usage Instructions
All scrapers follow the base structure outlined in [base_scraper.py](Scraper_master/base_scraper.py), and common utilities can be found in [utils.py](Scraper_master/utils.py). These are some common features about the scrapers:
- All links are compared to a root or base URL to ensure the webpage remains in a relevant domain (ex. for a root URL of berkeley.edu, berkeley.edu/about will be scraped, but youtube.com will not).
- Each conversion creates metadata file in the .yaml format that contains the URL of the page.

## General websites: scrape_header.py
Beginning from a root URL, the scraper runs depth-first-search. On the current page, a set is created, and all unique links that are found will be processed. This process continues for all the links in the set. The scraper can handle both HTML and PDF formats - if it encounters a webpage, it will be converted from HTML to MD, and if it encounters a PDF, it will download the PDF file. 

### Usage Instructions:
1. Update `url`, `root_regex`, and `root_filename` under the main function accordingly. `url` is the webpage where the scraping will begin, `root_regex` is the string that all further URLS will be compared to before being added to the traversal set, and `root_filename` will be the name of the root file. 
2. Next, update `content_tags`. `content_tags` is currently the result of the `match_tags` function, which will take a URL, and map it to the correct content tags specified by the dictionary `content_tags_dict`. The dictionary currently contains several examples of content tags for certain URLs, but the following pictures will illustrate how you can determine content tags for a website you need to scrape.

First, open up the the developer tools on your page of interest (MacOS: Command + Option + C, Windows: Control + Shift + C, or right-click --> inspect). Your page should look something like this:

![My Image](images/page_inspect.png)

Then, you can either hover over the section of the page that you want to scrape, or click open the dropdowns in the control panel until you find the correct area. It should look something like this:

![My Image](images/content_inspect.png)

Then, you will need to copy the element, in this case  `<div dir='ltr' id='page' lang='en'>`  and put it into the following format:
`[('div', {'id': 'page', 'lang': 'en', 'dir': 'ltr'})]`. If you have multiple elements that you want to scrape, you can add another tuple in the list in the same format. You can then add this into the `content_tags_dict`, or directly change the `content_tags` variable. 

3. In the directory you want your root file to be, run `python3 scrape_header.py`

## Markdown websites: scrape_md.py
The markdown scraper utilizes the "nav" section of mkdocs.yml file in a GitHub repo to run DFS. Based on the "nav" section structure and the base URL, it recursively fetch and save Markdown files.

### Usage Instructions:
1. Update `root_filename` to the name you want for your root folder
2. Update `site_url` to the URL of the website
3. Update `github_url` to the URL of the GitHub repo, specifically the link to the mkdocs.yml file. 
4. In the directory you want your root file to be, run `python3 scrape_md.py`

## RST websites: scrape_rst.py
The RST scraper uses the toctree from a GitHub repo to run DFS. It fetches the content of the current file, extracts links from the toctree, and then recursively processses each link. 

### Usage Instructions:
1. Update `filename` to the name you want for your root folder
2. Update `doc_url` to the URL of the website
3. Update `github_url` to the URL of the GitHub repo, specifically the link that contains the toctree (commonly found in `index.rst`)
4. In the directory you want your root file to be, run `python3 scrape_rst.py`

## PDFs: scrape_pdf.py
Downloads a PDF file given the link. 

### Usage Instructions:
1. Update `pdf_url` to the PDF URL that you want to download
2. Update the "name" string under the call to `content_extract` to what you want to name the PDF 
3. In the directory you want your PDF file to be, run `python3 scrape_pdf.py`

## Videos: scrape_vid.py
Given a playlist URL, the scraper will retrieve all of the playlist videos' URLS and download them in the specified directory. 

### Usage Instructions:
1. Update `base_path` to the name you want for your root folder
2. Update `playlist_url` to the name of the YouTube playlist you want to scrape
3. In the directory you want your root folder to be, run `python3 scrape_vid.py`

Note: Scraping is also supported for Ed, however this is done differently than the scrapers above, so please follow separate instructions to scrape the Ed forum. 

## End Results
- After running the scrapper, you will get a folder with this following tree structure.
  ```
  ├── MonashDataFluency
  │   ├── A (brief) Python refresher
  │   │   ├── section-0-brief-python-refresher.md
  │   │   ├── section-0-brief-python-refresher.md_metadata.yml
  │   ├── API based scraping
  │   │   ├── section-3-API-based-scraping.md
  │   │   ├── section-3-API-based-scraping.md_metadata.yaml
  │   ├── Getting started
  │   │   ├── index.md
  │   │   ├── index.md_metadata.yaml
  │   ├── HTML based scraping
  │   │   ├── section-2-HTML-based-scraping.md
  │   │   ├── section-2-HTML-based-scraping.md_metadata.yaml
  │   ├── Introduction to Web scraping
  │   │   ├── section-1-intro-to-web-scraping.md
  │   │   ├── section-1-intro-to-web-scraping.md_metadata.yaml
  │   ├── Legal and Ethical Considerations
  │   │   ├── section-5-legal-and-ethical-considerations.md
  │   │   ├── section-5-legal-and-ethical-considerations.md_metadata.yaml
  │   ├── References
  │   │   ├── section-7-references.md
  │   │   ├── section-7-references.md_metadata.yaml
  │   ├── Wrangling and Analysis
  │   │   ├── section-4-wrangling-and-analysis.md
  │   │   ├── section-4-wrangling-and-analysis.md_metadata.yaml
  ```

  This is an example of the result of running the entire code. It forms a tree structure of the entire website from the root webpage `MonashDataFluency`. Each subfolder will have some content extracted as well as a metadata file in the .yaml format. 

Now that you already have your documents ready, it's time to convert them into embeddings. 
