from pathlib import Path

from file_conversion_router.services.tai_MinerU_service.utils.convert import process_pdf


def convert_pdf_to_md_by_MinerU(input_pdf_path: Path, output_dir_path: Path) -> Path:
    return process_pdf(input_pdf_path, output_dir_path)
