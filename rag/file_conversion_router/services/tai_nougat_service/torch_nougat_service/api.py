from pathlib import Path

from . import run_nougat
from ..config_nougat.nougat_config import NougatConfig


def convert_pdf_to_mmd(input_pdf_path: Path, output_dir_path: Path) -> None:
    config = NougatConfig(
        pdf_paths=[input_pdf_path],
        output_dir=output_dir_path,
    )
    run_nougat(config)
