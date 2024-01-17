# scrapper
- We have scraper for different formats.
  1) Mardown
  2) Rst
  3) Plain
- What a scrapping tool does in general is it converts a particullar page of a document into a tree structure accordingly to it's heading levels.
```
(Table of Contents)
Quick start package installation (h1)
--Before you begin (h2)
----Windows (h3)
----Linux (h3)
--CARLA installation (h2)
----A. Debian CARLA installation (h3)
----B. Package installation (h3)
--Import additional assets (h2)
--Install client library (h2)
----CARLA versions prior to 0.9.12 (h3)
----CARLA 0.9.12+ (h3)
--Running CARLA (h2)
------Command-line options (h4)
--Updating CARLA (h2)
--Follow-up (h2)
```
## Folders for scraper
- In every scraping folder, there will be a code called `scrape.py` that is the place you will use to scrape your documents. 
- `Scraper_header`: 
  In order to use this folder you need edit some variables accordingly to your needs:
  ```
  # TODO
  url = "https://wiki.ros.org/ROS/Tutorials/"
  root = "https://wiki.ros.org/ROS/Tutorials/"
  root_regex = r"^https://wiki.ros.org/ROS/Tutorials/"
  root_filename = "ROS"
  ```
  - `url`: the url where your webside starts
  - `root`: the url where you want your website to recurse until
  - `root_regex`: the format of the url that it needs to follow
  - `root_filename`: the filename you want to store all your data
- `Scraper_md`:
  In order to use this folder you need to provide the url to the folder of mkdocs.yaml of the repo.
  `url="https://github.com/carla-simulator/carla/blob/master/mkdocs.yml"`
- `Scraper_rst`:
  In order to use this folder you will need to edit some variables accordingly to your needs
  ```
  mkdir('numpy')
  os.chdir('numpy')
  starting='index'
  url=f"https://github.com/numpy/numpy/blob/main/doc/source/index.rst?plain=1"
  ```
  - mkdir and os.chdir is just setting your directory name
  - `starting`: the name of your first rst file. (It's usually index)
  - `url`: the url to the index.rst of the repo. Notice you have to add `?plain=1` to the end of the url as that obtains th raw code of the rst file. 
