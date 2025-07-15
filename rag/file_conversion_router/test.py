from file_conversion_router.classes.page import Page
from pathlib import Path


def test_merge_functionality(
    input_path="/Users/yyk956614/tai/rag/output/md/01-Welcome_1pp/01-Welcome_1pp.md",
):
    with open(input_path, "r", encoding="utf-8") as input_file:
        content = input_file.read()
    content = {"text": content}

    page = Page(
        pagename="TestPage",
        content=content,
        filetype="pdf",
        page_url="http://example.com",
        mapping_json_path=Path(
            "/Users/yyk956614/tai/rag/output/md/01-Welcome_1pp/01-Welcome_1pp_content_list.json"
        ),
    )

    page.page_seperate_to_segments()
    page.tree_print()
    original_recursive_separate = page.recursive_separate
    page.recursive_separate = lambda text, token_limit=400: original_recursive_separate(
        text, token_limit=400
    )
    chunks = page.tree_segments_to_chunks()

    for idx, chunk in enumerate(chunks, start=1):
        print(f"Chunk {idx}:")
        print("Title:", chunk.titles)
        print("Page Num:", chunk.page_num)
        print("Content snippet:", chunk.content[:100], "...")
        print("-" * 40)


if __name__ == "__main__":
    test_merge_functionality()
