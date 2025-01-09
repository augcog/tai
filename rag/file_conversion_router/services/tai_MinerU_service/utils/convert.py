import os
from magic_pdf.data.data_reader_writer import FileBasedDataWriter, FileBasedDataReader
from magic_pdf.data.dataset import PymuDocDataset
from magic_pdf.model.doc_analyze_by_custom_model import doc_analyze
from pathlib import Path

def process_pdf_to_markdown(input_pdf_path: Path, output_dir: Path) -> None:
    """
    Processes a PDF file and outputs markdown and images to the specified output directory.

    Args:
        pdf_file_name (str): The path to the PDF file.
        output_dir (str): The directory where markdown and images will be saved.
    """
    # Get file name without extension
    name_without_suff = os.path.splitext(os.path.basename(str(input_pdf_path)))[0]

    # Prepare output directories
    local_image_dir = os.path.join(output_dir, "images")
    local_md_file_path = os.path.join(output_dir, f"{name_without_suff}.md")

    # Create directories if they don’t exist
    os.makedirs(local_image_dir, exist_ok=True)
    os.makedirs(output_dir, exist_ok=True)

    # Initialize data writers
    image_writer = FileBasedDataWriter(local_image_dir)
    md_writer = FileBasedDataWriter(output_dir)

    # Read PDF bytes
    reader = FileBasedDataReader("")
    pdf_bytes = reader.read(str(input_pdf_path))

    # Process the PDF
    print("Starting PDF processing...")
    ds = PymuDocDataset(pdf_bytes)

    # Run the analysis, OCR, and markdown dump
    ds.apply(doc_analyze, ocr=True).pipe_ocr_mode(image_writer).dump_md(md_writer, local_md_file_path, "images")

    print(f"Markdown saved in: {local_md_file_path}")
    print(f"Images saved in: {local_image_dir}")

# Example usage
if __name__ == "__main__":
    pdf_file = "/tests/test_rag/data/unit_tests/pdf/input/61a-sp24-mt1.pdf"
    process_pdf_to_markdown(pdf_file, output_dir="output")
