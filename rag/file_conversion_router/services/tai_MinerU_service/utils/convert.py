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
    output_dir = output_dir.resolve().parent  # Ensure output_dir is the parent directory

    # Extract the PDF file name (without extension)
    output_name = pdf_path.stem
    # Define subdirectories
    images_dir = output_dir / "images"  # Directory for storing OCR-generated images
    markdown_dir = output_dir  # Directory for storing Markdown & JSON files
    pdf_dir = output_dir / "PDFS"
    json_dir = output_dir / "JSONS"
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

    # Generate a model inference visualization PDF
    infer_result.draw_model(str(pdf_dir / f"{output_name}_model.pdf"))

    # Generate layout and text span visualization PDFs
    pipe_result.draw_layout(str(pdf_dir / f"{output_name}_layout.pdf"))
    pipe_result.draw_span(str(pdf_dir / f"{output_name}_spans.pdf"))

    # Export Markdown (stored in a separate subdirectory)
    md_content = pipe_result.get_markdown("images")
    pipe_result.dump_md(md_writer, f"{output_name}.md", "images")

    # Export structured JSON data
    content_list = pipe_result.get_content_list("images")
    pipe_result.dump_content_list(md_writer, json_dir / f"{output_name}_content_list.json", "images")

    # Export intermediate JSON data
    middle_json = pipe_result.get_middle_json()
    pipe_result.dump_middle_json(md_writer, json_dir / f"{output_name}_middle.json")
    md_file_path = markdown_dir / f"{output_name}.md"
    json_file_path = json_dir / f"{output_name}_content_list.json"

    with open(md_file_path, 'r', encoding='utf-8') as f_md:
        md_content = f_md.read()

    # 2. 读取 JSON 文件内容（假设文件名为 file.json）
    with open(json_file_path, 'r', encoding='utf-8') as f_json:
        json_list = json.load(f_json)

    # 3. 将 JSON 对象按 page_idx 分组
    pages = collections.defaultdict(list)
    for item in json_list:
        page_idx = item.get("page_idx")
        pages[page_idx].append(item)

    # 用于存储每个页面的起始行号，结构为 {page_idx: start_line}
    pages_start_lines = {}

    # 4. 遍历每个页面，寻找该页面中各文本段在 Markdown 文件中的最早位置
    for page_idx, items in pages.items():
        page_start_line = None  # 用于保存该页面中最小的行号
        for item in items:
            text_snippet = item.get("text", "").strip()
            # 如果文本为空，则跳过
            if not text_snippet:
                continue

            # 在 Markdown 文件中查找该文本段的位置
            pos = md_content.find(text_snippet)
            if pos == -1:
                # 没有找到匹配的文本，则跳过（或记录日志）
                continue

            # 计算该文本段在 Markdown 文件中的起始行号
            line_num = md_content[:pos].count('\n') + 1

            # 更新页面的起始行（取最小的行号）
            if page_start_line is None or line_num < page_start_line:
                page_start_line = line_num

        # 如果该页面有匹配到文本，记录起始行；否则可置为 None 或其他默认值
        pages_start_lines[page_idx] = page_start_line
    yaml_file_path = md_file_path.with_name(md_file_path.stem + '_page_info.yaml')
    # 5. 将 pages_start_lines 写入一个 YAML 文件（例如 pages.yaml）
    with open(yaml_file_path, 'w', encoding='utf-8') as f_yaml:
        # 使用 safe_dump，并允许 Unicode 字符（如中文）写入
        yaml.safe_dump(pages_start_lines, f_yaml, allow_unicode=True)

    print("YAML 文件写入成功，内容如下：")
    print(yaml.safe_dump(pages_start_lines, allow_unicode=True))


