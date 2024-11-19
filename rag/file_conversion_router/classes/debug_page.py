from pathlib import Path
from rag.file_conversion_router.classes.page import Page

# Define paths
md_path = "output_tmp/expected_output/filename/filename.md"
metadata_path = Path("output_tmp/expected_output/filename/filename_metadata.yaml")  # Replace with your actual metadata path

# Load markdown content
with open(md_path, "r", encoding="utf-8") as input_file:
    content_text = input_file.read()

markdown_content = {
    "text": content_text
}

# Instantiate Page object
page = Page(
    pagename="SamplePage",
    content=markdown_content,
    filetype="md",
    page_url="https://ucb-ee106.github.io/106b-sp23site/assets/disc/Discussion_1_Dynamical_Systems_Solution.pdf",
    metadata_path=metadata_path
)

# Process content into chunks
page.to_chunk()
page.chunks_to_pkl(Path("output_tmp/expected_output/filename/filename.pkl"))

# Print header tree
header_tree = page.print_header_tree()
print(header_tree)

# Call the debug function
# page.to_chunk()
# page.page_seperate_to_segments()
# page.tree_print()
# page.debug_page_num()
# page.find_empty_content_headers()
page.print_chunk_content()
# page.debug_empty_headers()

# page.tree_print()
# page.print_header_tree()
# page.debug_chunks()