# Scrape_ed

## TO USE/TEST
- run `python3 scrape.py` from the `tai/rag/scraper/Scrape_ed` directory, populating
    - `debug.md`: for debugging, knowledge base with pruned out information
    - `outputkb.md`: knowledge base with only (mostly) useful information

## scrape.py
- standardized post scraping function(s) from an ed's json
    - KB case: 
        - Takes in all posts! Has separate logic for announcement/question posts
        - All comments are recursively handled by the separate `process_comments` function
    - Q/A case:
        - Takes in all posts! However, non-question posts texts are pruned out
        - All comments are recursively handled by the separate `process_comments_qa` function

## filter.py
- filters json into two separate markdowns, each with mutually exclusive useful/less useful info
    - Currently only has KB functionality (Q/A case coming soon!!)
    - Prioritizes minimizing false negatives over false negatives (can sometimes "overprune")
    - Filter features:
        - takes out personalized posts (requests, specific queries, etc.)
        - prunes unpopular (commentless, voteless) posts
        - and more!
- View code for more details! 