from pathlib import Path
from magic_pdf.data.data_reader_writer import FileBasedDataWriter, FileBasedDataReader
from magic_pdf.data.dataset import PymuDocDataset
from magic_pdf.model.doc_analyze_by_custom_model import doc_analyze
from magic_pdf.config.enums import SupportedPdfParseMethod
import json
import collections
import yaml


def process_pdf(pdf_path: Path, output_dir: Path):
    """
    Process a PDF file, automatically determine whether to use OCR or text mode,
    and generate Markdown, JSON, and visualized PDF results.

    :param pdf_path: The path to the PDF file to be processed (Path object).
    :param output_dir: The parent directory where output files will be stored (Path object).
    """
    # Ensure pdf_path and output_dir are Path objects
    pdf_path = pdf_path.resolve()
    output_dir = (
        output_dir.resolve().parent
    )  # Ensure output_dir is the parent directory

    # Extract the PDF file name (without extension)
    output_name = pdf_path.stem
    # Define subdirectories
    images_dir = output_dir / "images"  # Directory for storing OCR-generated images
    markdown_dir = output_dir  # Directory for storing Markdown & JSON files
    pdf_dir = output_dir
    json_dir = output_dir
    # Create required directories if they do not exist
    for directory in [output_dir, images_dir, markdown_dir]:
        directory.mkdir(parents=True, exist_ok=True)

    # Create file writers
    image_writer = FileBasedDataWriter(str(images_dir))
    md_writer = FileBasedDataWriter(str(markdown_dir))  # Store Markdown separately
    reader = FileBasedDataReader("")

    # Read the PDF file
    pdf_bytes = reader.read(str(pdf_path))

    # Create a dataset instance
    ds = PymuDocDataset(pdf_bytes)

    # Process the PDF (OCR or text mode)
    if ds.classify() == SupportedPdfParseMethod.OCR:
        infer_result = ds.apply(doc_analyze, ocr=True)
        pipe_result = infer_result.pipe_ocr_mode(image_writer)
    else:
        infer_result = ds.apply(doc_analyze, ocr=False)
        pipe_result = infer_result.pipe_txt_mode(image_writer)

    # Generate layout and text span visualization PDFs
    pipe_result.draw_layout(str(pdf_dir / f"{output_name}_layout.pdf"))
    pipe_result.draw_span(str(pdf_dir / f"{output_name}_spans.pdf"))

    # Export Markdown (stored in a separate subdirectory)
    md_content = pipe_result.get_markdown("images")
    pipe_result.dump_md(md_writer, f"{output_name}.md", "images")

    # Export structured JSON data
    content_list = pipe_result.get_content_list("images")
    pipe_result.dump_content_list(
        md_writer, json_dir / f"{output_name}_content_list.json", "images"
    )

    md_file_path = markdown_dir / f"{output_name}.md"
    json_file_path = json_dir / f"{output_name}_content_list.json"

    with open(md_file_path, "r", encoding="utf-8") as f_md:
        md_content = f_md.read()

    with open(json_file_path, "r", encoding="utf-8") as f_json:
        json_list = json.load(f_json)

    # Create a defaultdict to group items by page index
    pages = collections.defaultdict(list)
    for item in json_list:
        page_idx = item.get("page_idx")
        pages[page_idx].append(item)

    # Read Markdown content as a list of lines
    md_lines = md_content.split("\n")

    # List to store structured page info
    pages_start_lines = []

    for page_idx, items in pages.items():
        page_start_line = None  # Track the first occurrence of a page's content

        for item in items:
            text_snippet = item.get("text", "").strip()
            if not text_snippet:
                continue  # Skip empty text items

            # Find the first line where the snippet appears
            for line_idx, line in enumerate(md_lines):
                if text_snippet in line:
                    if page_start_line is None or line_idx < page_start_line:
                        page_start_line = line_idx + 1  # Convert to 1-based index
                    break  # Stop after finding the first match in the file

        # Append structured data to the list
        if page_start_line is not None:
            pages_start_lines.append(
                {"page_num": page_idx, "start_line": page_start_line}
            )

    # Define YAML file path
    yaml_file_path = md_file_path.with_name(md_file_path.stem + "_page_info.yaml")

    # Save structured page info to YAML
    with open(yaml_file_path, "w", encoding="utf-8") as f_yaml:
        yaml.safe_dump({"pages": pages_start_lines}, f_yaml, allow_unicode=True)

    # Print the formatted YAML output
    print(yaml.safe_dump({"pages": pages_start_lines}, allow_unicode=True))
