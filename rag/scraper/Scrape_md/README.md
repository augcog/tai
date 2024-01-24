# Scraper_md  
In order to use this folder you need to provide the url to the folder of mkdocs.yaml of the repo and the name of the folder you want to save your documents at.  
  ```
  create_and_enter_dir('carla')
  url="https://github.com/carla-simulator/carla/blob/master/mkdocs.yml"
  ```
- After you have set up the variables you can run `python3 scrape.py` and it will start scraping the website.