from pathlib import Path
from rag.file_conversion_router.classes.page import Page

# Define paths
md_path = "/Users/yyk956614/tai/tests/test_rag/data/integrated_tests/expected_output_folder1_plain_folder_3_pdfs/hw05_3pages/hw05_3pages.md"
metadata_path = Path("/Users/yyk956614/tai/output_tmp/expected_output/debug/filename/filename.yaml")  # Replace with your actual metadata path

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
    page_url="'https://ucb-ee106.github.io/106b-sp23site/assets/disc/Discussion_1_Dynamical_Systems_Solution.pdf",
    metadata_path=metadata_path
)

# Process content into chunks
page.to_chunk()
page.chunks_to_pkl(Path("/Users/yyk956614/tai/tests/test_rag/data/integrated_tests/expected_output_folder1_plain_folder_3_pdfs/hw05_3pages/hw05_3pages.pkl"))

# Print header tree
header_tree = page.print_header_tree()
print(header_tree)

# Call the debug function
page.debug_page_num()

