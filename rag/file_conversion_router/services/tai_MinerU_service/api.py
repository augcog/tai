from utils.convert import process_pdf_to_markdown
from pathlib import Path


def convert_pdf_to_mmd(input_pdf_path: Path, output_dir_path: Path) -> None:
    process_pdf_to_markdown(input_pdf_path, output_dir_path)

# Example usage
if __name__ == "__main__":
    pdf_file = "/home/roar-tai-1/Desktop/yk/tai/tests/test_rag/data/unit_tests/pdf/input/61a-sp24-mt1.pdf"
    output_dir = "output"
    convert_pdf_to_mmd(pdf_file, output_dir)