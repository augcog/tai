import re
from pathlib import Path
from typing import List, Tuple

# Compile the regex pattern for header recognition for improved performance
HEADER_PATTERN = re.compile(r'^#+')


def parse_headers(markdown_content: str) -> List[Tuple[int, str]]:
    """
    Parses the markdown content to find headers and determines their levels.

    Args:
        markdown_content (str): The content of the markdown file.

    Returns:
        List[Tuple[int, str]]: A list of tuples, where each tuple contains the header level and the header text.
    """
    headers = []
    in_code_block = False
    for line in markdown_content.splitlines():
        if '```' in line.strip():  # Toggle state on encountering code block delimiters
            in_code_block = not in_code_block
        if not in_code_block and line.startswith('#'):
            level = len(HEADER_PATTERN.match(line).group())
            title = line.strip('# ').strip()
            headers.append((level, title))
    return headers


def generate_header_tree(headers: List[Tuple[int, str]]) -> str:
    """
    Generates a markdown formatted string that represents the tree structure of markdown headers.

    Args:
        headers (List[Tuple[int, str]]): A list of headers with their respective levels and texts.

    Returns:
        str: A string formatted in markdown representing the hierarchical tree of headers.
    """
    content = ["# Table of Contents"]
    last_level = 0
    for level, title in headers:
        if level > last_level:
            content.append('  ' * (level - 1) + f'- {title}')
        elif level == last_level:
            content.append('  ' * (level - 1) + f'- {title}')
        else:
            content.append('  ' * (level - 1) + f'- {title}')
        last_level = level
    return "\n".join(content)


class MarkdownToMarkdownConverter:
    """Converts a Markdown file to a new Markdown file with a structured tree of its headers."""

    def __init__(self, input_path: Path, output_path: Path):
        self.input_path = input_path
        self.output_path = output_path

    def perform_conversion(self) -> None:
        """
        Perform the conversion process from one markdown to another with a header tree.
        """
        markdown_content = self.input_path.read_text(encoding='utf-8')
        headers = parse_headers(markdown_content)
        tree_markdown = generate_header_tree(headers)
        self.output_path.write_text(tree_markdown, encoding='utf-8')

# Example usage:
converter = MarkdownToMarkdownConverter(
    Path('/Users/jingchaozhong/Desktop/quick_folders/Cal Study/roar-ai/roarai/rag/test/data/unit_tests/md/input/section-0-brief-python-refresher.md'),
    Path('/Users/jingchaozhong/Desktop/quick_folders/Cal Study/roar-ai/roarai/rag/test/data/unit_tests/md/expected_output/test.md')
)
converter.perform_conversion()
