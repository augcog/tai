from pathlib import Path
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page

class PythonConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        """Converts a Python file to a Markdown file by formatting it as a code block."""
        
        output_path = output_path.with_suffix(".md")
        
        with open(input_path, "r") as input_file, open(output_path, "w") as output_file:
            content = input_file.read()
            markdown_content = f"```python\n{content}\n```"  
            output_file.write(markdown_content)
        
        return output_path