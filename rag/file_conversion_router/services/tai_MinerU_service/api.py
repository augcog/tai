from pathlib import Path

from file_conversion_router.services.tai_MinerU_service.utils.convert import parse_doc


def convert_pdf_to_md_by_MinerU(input_pdf_path: Path, output_dir_path: Path) -> Path:
    return parse_doc(input_pdf_path, output_dir_path)
