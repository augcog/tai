import re
from typing import Dict, Tuple, List

def count_consecutive_hashes(line: str) -> int:
    """Count consecutive '#' characters at the start of the string."""
    return len(line) - len(line.lstrip('#'))

def extract_headers_and_content(md_content: List[str]) -> Dict[Tuple[str, int], str]:
    """Extract headers and corresponding content from Markdown content."""
    headers_content = {}
    current_header = None
    in_code_block = False

    for line in md_content:
        stripped_line = line.strip()

        # Toggle in/out of code blocks
        if "```" in stripped_line:
            in_code_block = not in_code_block

        if in_code_block or not line.startswith('#'):
            if current_header:
                headers_content[current_header] += line
            continue

        # Process headers outside of code blocks
        header_level = count_consecutive_hashes(line)
        header_text = line.strip('#').strip()
        current_header = (header_text, header_level)
        headers_content[current_header] = ""

    return headers_content

def generate_tree_structure(headers_content: Dict[Tuple[str, int], str]) -> str:
    """Generate a formatted tree structure of headers."""
    result = []
    for (header, level), content in headers_content.items():
        indent = '    ' * (level - 1)  # 4 spaces per level indentation
        header_tag = f"(h{level})"
        result.append(f"{indent}{header} {header_tag}")
        result.append(content.strip())
        result.append('-' * 80)  # Separator after each section
    return '\n'.join(result)

def process_markdown_file(input_filepath: str, output_filepath: str) -> None:
    """Read a Markdown file, extract its structure, and write to a new text file."""
    with open(input_filepath, 'r', encoding='utf-8') as file:
        md_content = file.readlines()

    headers_content = extract_headers_and_content(md_content)
    structured_content = generate_tree_structure(headers_content)

    with open(output_filepath, 'w', encoding='utf-8') as file:
        file.write(structured_content)

# Example usage:

input_path = '/Users/jingchaozhong/Desktop/quick_folders/Cal Study/roar-ai/roarai/rag/test/data/unit_tests/md/input/section-0-brief-python-refresher.md'
output_path = '/Users/jingchaozhong/Desktop/quick_folders/Cal Study/roar-ai/roarai/rag/test/data/unit_tests/md/expected_output/test.md'

process_markdown_file(input_path, output_path)
