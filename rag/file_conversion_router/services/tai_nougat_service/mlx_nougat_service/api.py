from pathlib import Path

from . import main


def convert_pdf_to_mmd(input_pdf_path: Path, output_dir_path: Path) -> None:
    main(input_pdf_path, output_dir_path)
