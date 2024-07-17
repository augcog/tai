# Scrape_rst
In order to use this folder you will need to edit some variables in `scrape.py` accordingly to your needs:
  ```
  mkdir('numpy')
  os.chdir('numpy')
  starting='index'
  url=f"https://github.com/numpy/numpy/blob/main/doc/source/index.rst?plain=1"
  ```
- mkdir and os.chdir is just setting your directory name
- `starting`: the name of your first rst file. (It's usually index)
- `url`: the url to the index.rst of the repo. Notice you have to add `?plain=1` to the end of the url as that obtains the raw code of the rst file.
- After you have set up the variables you can run `python3 scrape.py` and it will start scraping the website. If you want to convert this folder into markdown, make the changes in `scrape_md.py` and run `python3 scrape_md.py` and it will produce the tree structure but in markdown format.