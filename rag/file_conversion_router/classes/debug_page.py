from rag.file_conversion_router.classes.page import Page

md_path = "output_tmp/expected_output/filename/filename.md"  

with open(md_path, "r", encoding="utf-8") as input_file:
    content_text = input_file.read()

markdown_content = {
    "text": content_text  
}

page = Page(
    pagename="SamplePage",          
    content=markdown_content,       
    filetype="md",                  
    page_url="http://example.com"  
)
page.to_chunk() 

header_tree = page.print_header_tree()
print(header_tree)
page.chunks_to_pkl("output_chunks.pkl")
page.to_file("output_tree.txt")
page.debug_page_num()


