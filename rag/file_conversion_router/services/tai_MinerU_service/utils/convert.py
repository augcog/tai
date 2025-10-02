# Copyright (c) Opendatalab. All rights reserved.
import copy
import json
import os
from pathlib import Path
import re

from loguru import logger

from mineru.cli.common import convert_pdf_bytes_to_bytes_by_pypdfium2, prepare_env, read_fn
from mineru.data.data_reader_writer import FileBasedDataWriter
from mineru.utils.draw_bbox import draw_layout_bbox, draw_span_bbox
from mineru.utils.enum_class import MakeMode
from mineru.backend.vlm.vlm_analyze import doc_analyze as vlm_doc_analyze
from mineru.backend.pipeline.pipeline_analyze import doc_analyze as pipeline_doc_analyze
from mineru.backend.pipeline.pipeline_middle_json_mkcontent import union_make as pipeline_union_make
from mineru.backend.pipeline.model_json_to_middle_json import result_to_middle_json as pipeline_result_to_middle_json
from mineru.backend.vlm.vlm_middle_json_mkcontent import union_make as vlm_union_make
from mineru.utils.models_download_utils import auto_download_and_get_model_root_path


def do_parse(
        output_dir,  # Output directory for storing parsing results
        pdf_file_path: list[str],  # List of PDF file names to be parsed
        pdf_bytes_list: list[bytes],  # List of PDF bytes to be parsed
        p_lang_list: list[str],  # List of languages for each PDF, default is 'ch' (Chinese)
        backend="pipeline",  # The backend for parsing PDF, default is 'pipeline'
        parse_method="auto",  # The method for parsing PDF, default is 'auto'
        formula_enable=True,  # Enable formula parsing
        table_enable=True,  # Enable table parsing
        server_url=None,  # Server URL for vlm-sglang-client backend
        f_draw_layout_bbox=False,  # Whether to draw layout bounding boxes
        f_draw_span_bbox=False,  # Whether to draw span bounding boxes
        f_dump_md=True,  # Whether to dump markdown files
        f_dump_middle_json=False,  # Whether to dump middle JSON files
        f_dump_model_output=False,  # Whether to dump model output files
        f_dump_orig_pdf=False,  # Whether to dump original PDF files
        f_dump_content_list=True,  # Whether to dump content list files
        f_make_md_mode=MakeMode.MM_MD,  # The mode for making markdown content, default is MM_MD
        start_page_id=0,  # Start page ID for parsing, default is 0
        end_page_id=None,
        file_name=None # End page ID for parsing, default is None (parse all pages until the end of the document)
):
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)

    if backend == "pipeline":
        for idx, pdf_bytes in enumerate(pdf_bytes_list):
            new_pdf_bytes = convert_pdf_bytes_to_bytes_by_pypdfium2(pdf_bytes, start_page_id, end_page_id)
            pdf_bytes_list[idx] = new_pdf_bytes

        infer_results, all_image_lists, all_pdf_docs, lang_list, ocr_enabled_list = pipeline_doc_analyze(
            pdf_bytes_list, p_lang_list, parse_method=parse_method,
            formula_enable=formula_enable, table_enable=table_enable
        )

        for idx, model_list in enumerate(infer_results):
            model_json = copy.deepcopy(model_list)
            # Create image subdirectory for this specific file
            local_image_dir = os.path.join(output_dir, f"{file_name}_images")
            os.makedirs(local_image_dir, exist_ok=True)
            image_writer = FileBasedDataWriter(local_image_dir)

            # Main output writer for the output directory
            main_writer = FileBasedDataWriter(output_dir)

            images_list = all_image_lists[idx]
            pdf_doc = all_pdf_docs[idx]
            _lang = lang_list[idx]
            _ocr_enable = ocr_enabled_list[idx]
            middle_json = pipeline_result_to_middle_json(
                model_list, images_list, pdf_doc, image_writer, _lang, _ocr_enable, formula_enable
            )

            pdf_info = middle_json["pdf_info"]
            pdf_bytes = pdf_bytes_list[idx]

            # Define main md file path
            md_file_path = os.path.join(output_dir, f"{file_name}.md")
            content_list_path = os.path.join(output_dir, f"{file_name}_content_list.json")

            if f_draw_layout_bbox:
                draw_layout_bbox(pdf_info, pdf_bytes, output_dir, f"{file_name}_layout.pdf")

            if f_draw_span_bbox:
                draw_span_bbox(pdf_info, pdf_bytes, output_dir, f"{file_name}_span.pdf")

            if f_dump_orig_pdf:
                main_writer.write(f"{file_name}_origin.pdf", pdf_bytes)

            if f_dump_md:
                image_dir = os.path.basename(local_image_dir)
                md_content_str = pipeline_union_make(pdf_info, f_make_md_mode, image_dir)
                main_writer.write_string(md_file_path, md_content_str)

            if f_dump_content_list:
                image_dir = os.path.basename(local_image_dir)
                content_list = pipeline_union_make(pdf_info, MakeMode.CONTENT_LIST, image_dir)
                cleaned_content_list = clean_unicode_surrogates(content_list)
                # Save JSON as {md_file_path}_content_list.json
                with open(content_list_path, 'w', encoding='utf-8') as f:
                    json.dump(cleaned_content_list, f, ensure_ascii=False, indent=4)

            if f_dump_middle_json:
                main_writer.write_string(
                    f"{file_name}_middle.json",
                    json.dumps(middle_json, ensure_ascii=False, indent=4),
                )

            if f_dump_model_output:
                main_writer.write_string(
                    f"{file_name}_model.json",
                    json.dumps(model_json, ensure_ascii=False, indent=4),
                )

            logger.info(f"Markdown saved to: {md_file_path}")
            if f_dump_content_list:
                logger.info(f"Content list saved to: {content_list_path}")

    else:
        if backend.startswith("vlm-"):
            backend = backend[4:]

        f_draw_span_bbox = False
        parse_method = "vlm"

        for idx, pdf_bytes in enumerate(pdf_bytes_list):
            pdf_bytes = convert_pdf_bytes_to_bytes_by_pypdfium2(pdf_bytes, start_page_id, end_page_id)

            # Create image subdirectory for this specific file
            local_image_dir = os.path.join(output_dir, f"{file_name}_images")
            os.makedirs(local_image_dir, exist_ok=True)
            image_writer = FileBasedDataWriter(local_image_dir)

            # Main output writer for the output directory
            main_writer = FileBasedDataWriter(output_dir)

            middle_json, infer_result = vlm_doc_analyze(
                pdf_bytes, image_writer=image_writer, backend=backend, server_url=server_url
            )

            pdf_info = middle_json["pdf_info"]

            # Define main md file path
            md_file_path = os.path.join(output_dir, f"{file_name}.md")
            content_list_path = f"{md_file_path}_content_list.json"

            if f_draw_layout_bbox:
                draw_layout_bbox(pdf_info, pdf_bytes, output_dir, f"{file_name}_layout.pdf")

            if f_draw_span_bbox:
                draw_span_bbox(pdf_info, pdf_bytes, output_dir, f"{file_name}_span.pdf")

            if f_dump_orig_pdf:
                main_writer.write(f"{file_name}_origin.pdf", pdf_bytes)

            if f_dump_md:
                image_dir = os.path.basename(local_image_dir)
                md_content_str = vlm_union_make(pdf_info, f_make_md_mode, image_dir)
                main_writer.write_string(f"{file_name}.md", md_content_str)

            if f_dump_content_list:
                image_dir = os.path.basename(local_image_dir)
                content_list = vlm_union_make(pdf_info, MakeMode.CONTENT_LIST, image_dir)
                # Save JSON as {md_file_path}_content_list.json
                with open(content_list_path, 'w', encoding='utf-8') as f:
                    json.dump(content_list, f, ensure_ascii=False, indent=4)

            if f_dump_middle_json:
                main_writer.write_string(
                    f"{file_name}_middle.json",
                    json.dumps(middle_json, ensure_ascii=False, indent=4),
                )

            if f_dump_model_output:
                model_output = ("\n" + "-" * 50 + "\n").join(infer_result)
                main_writer.write_string(f"{file_name}_model_output.txt", model_output)

            logger.info(f"Markdown saved to: {md_file_path}")
            if f_dump_content_list:
                logger.info(f"Content list saved to: {content_list_path}")


def parse_doc(
        pdf_path: Path,
        output_folder: Path,
        lang="en",
        backend="pipeline",
        method="auto",
        server_url=None,
        start_page_id=0,
        end_page_id=None
):
    """
        Parameter description:
        pdf_path: Path to the PDF file to be parsed.
        output_folder: Output folder where the markdown file will be saved.
        lang: Language option, default is 'ch', optional values include['ch', 'ch_server', 'ch_lite', 'en', 'korean', 'japan', 'chinese_cht', 'ta', 'te', 'ka']。
            Input the languages in the pdf (if known) to improve OCR accuracy.  Optional.
            Adapted only for the case where the backend is set to "pipeline"
        backend: the backend for parsing pdf:
            pipeline: More general.
            vlm-transformers: More general.
            vlm-sglang-engine: Faster(engine).
            vlm-sglang-client: Faster(client).
            without method specified, pipeline will be used by default.
        method: the method for parsing pdf:
            auto: Automatically determine the method based on the file type.
            txt: Use text extraction method.
            ocr: Use OCR method for image-based PDFs.
            Without method specified, 'auto' will be used by default.
            Adapted only for the case where the backend is set to "pipeline".
        server_url: When the backend is `sglang-client`, you need to specify the server_url, for example:`http://127.0.0.1:30000`
        start_page_id: Start page ID for parsing, default is 0
        end_page_id: End page ID for parsing, default is None (parse all pages until the end of the document)

        Returns:
        Path: Path to the output markdown file
    """

    # Ensure output folder exists
    output_folder = Path(output_folder)

    # Get the PDF file name without extension
    file_name = pdf_path.name


    # Read the PDF file
    pdf_bytes = read_fn(pdf_path)
    pdf_path = str(pdf_path)


    # Create output directory for this specific file
    do_parse(
        output_dir=output_folder,
        pdf_file_path=[pdf_path],
        pdf_bytes_list=[pdf_bytes],
        p_lang_list=[lang],
        backend=backend,
        parse_method=method,
        server_url=server_url,
        start_page_id=start_page_id,
        end_page_id=end_page_id,
        file_name=file_name
    )

    # Return the path to the expected markdown file
    return output_folder / f"{file_name}.md"


def clean_unicode_surrogates(obj):
    """Recursively clean Unicode surrogate characters from any data structure"""
    if isinstance(obj, str):
        # Remove or replace surrogate characters
        # Option 1: Replace with intended emojis
        obj = obj.replace('\ud83e', '🦃')  # turkey emoji
        obj = obj.replace('\ud83d', '🐛')  # caterpillar emoji

        # Option 2: Remove any remaining surrogates
        obj = re.sub(r'[\ud800-\udfff]', '', obj)

        # Option 3: Encode/decode to ensure valid UTF-8
        obj = obj.encode('utf-8', 'ignore').decode('utf-8')

        return obj
    elif isinstance(obj, dict):
        return {key: clean_unicode_surrogates(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [clean_unicode_surrogates(item) for item in obj]
    else:
        return obj


if __name__ == '__main__':
    # args
    __dir__ = os.path.dirname(os.path.abspath(__file__))
    pdf_files_dir = os.path.join(__dir__, "pdfs")
    output_dir = os.path.join(__dir__, "output")
    pdf_suffixes = [".pdf"]
    image_suffixes = [".png", ".jpeg", ".jpg"]

    doc_path_list = []
    for doc_path in Path(pdf_files_dir).glob('*'):
        if doc_path.suffix in pdf_suffixes + image_suffixes:
            doc_path_list.append(doc_path)

    """如果您由于网络问题无法下载模型，可以设置环境变量MINERU_MODEL_SOURCE为modelscope使用免代理仓库下载模型"""
    # os.environ['MINERU_MODEL_SOURCE'] = "modelscope"

    """Use pipeline mode if your environment does not support VLM"""
    parse_doc(doc_path_list, output_dir, backend="pipeline")

    """To enable VLM mode, change the backend to 'vlm-xxx'"""
    # parse_doc(doc_path_list, output_dir, backend="vlm-transformers")  # more general.
    # parse_doc(doc_path_list, output_dir, backend="vlm-sglang-engine")  # faster(engine).
    # parse_doc(doc_path_list, output_dir, backend="vlm-sglang-client", server_url="http://127.0.0.1:30000")  # faster(client).