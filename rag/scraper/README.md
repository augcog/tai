# scrapper

## Folders for scraper
- Scrapper usually takes a root url or pdf and scrapes the entire website or pdf. 
- In every scraping folder, there will be a code called `scrape.py` that is the place you will use to scrape your documents. 
  - `scrape.py` will scrape the documents from the root url recursively until the entire website is scraped.  
- [Scrape_header](Scrape_header/): For general websites
- [Scrape_md](Scrape_md/): For websites that uses markdown
- [Scrape_rst](Scrape_rst/): For websites that uses rst
- [Scrape_pdf](Scrape_pdf/): For pdfs
- [Scrape_vid](Scrape_vid/): For videos

## End Results
- After running the scrapper, you will get a folder with this following tree structure that will be used in `rag/embedding_crate.py`.
  ```
  (rag) bot@botPC:~/roarai/rag/scraper/Scrape_md/carla$ tree .
  .
  ├── CARLA Ecosystem
  │   ├── ANSYS
  │   │   ├── ecosys_ansys.md
  │   │   ├── ecosys_ansys.pkl
  │   │   ├── ecosys_ansys_segment.txt
  │   │   └── ecosys_ansys_tree.txt
  │   ├── AWS
  │   │   ├── tuto_G_rllib_integration.md
  │   │   ├── tuto_G_rllib_integration.pkl
  │   │   ├── tuto_G_rllib_integration_segment.txt
  │   │   └── tuto_G_rllib_integration_tree.txt
  │   ├── CarSIM
  │   │   ├── tuto_G_carsim_integration.md
  │   │   ├── tuto_G_carsim_integration.pkl
  │   │   ├── tuto_G_carsim_integration_segment.txt
  │   │   └── tuto_G_carsim_integration_tree.txt
  ```
  This is an example of the result of running the entire code. It forms a tree structure of the entire website from the root webpage `Carla`.  
  - Each webpage will have an individual folder containing, `<webpage_name>.md`, `<webpage_name>_segment.txt`, and `<webpage_name>_md_tree.txt`.
    - `<webpage_name>.md`: This is the entire content of the webpage in markdown format.
    - `<webpage_name>_segment.txt`: This file contains all the headers and it's contents. 
    - `<webpage_name>_tree.txt`: This file contains the tree structure and the segments of the tree structure of the webpage.
      - Here is what a tree structure looks like in a webpage. 
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
    - Each segment would be the path from the root to the leaf node.
      - For example, the path to the node Linux would be `(h1) Quick Start Package Installation -> (h2) Before You Begin -> (h3) Linux`.
      - The purpose of grouping the documents in this tree structure is to allow the embedding model to understand the relationship between the headers. If the embedding model is only given the content of `(h3) Linux`, it would not know what it is related to nor how we get to the point `(h3) Linux`. By adding the segments from previous headers, it becomes complete information that explains: this information is about "installation", then continues with "steps you need to do before you begin", and how you begin in "Linux". 

Now that you already have your documents ready, it's time to convert them into embeddings. 
