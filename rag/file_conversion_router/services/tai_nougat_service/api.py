import logging

from .__init__ import run_nougat
from .nougat_config import NougatConfig

logging.basicConfig(level=logging.INFO)


def convert_pdf_to_mmd(config: NougatConfig) -> None:
    """Converts a PDF file to MMD format using TAI Nougat.
    """
    logging.info(
        "Initialized NougatConfig",
        extra={"config": config}
    )

    try:
        logging.info("Executing conversion process.")
        run_nougat(config)
        logging.info(
            "Conversion completed successfully",
            extra={"input_pdf_paths": config.pdf_paths, "output_dir_path": str(config.output_dir)}
        )
    except Exception as e:
        logging.error(
            "Conversion failed",
            exc_info=True,
            extra={"input_pdf_paths": config.pdf_paths, "output_dir_path": str(config.output_dir), "error": str(e)}
        )
        raise
