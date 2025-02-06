# import logging
#
# # from .__init__ import run_nougat
# from .config_nougat.tai_nougat_config import TAINougatConfig
#
# logging.basicConfig(level=logging.INFO)
#
#
# def convert_pdf_to_mmd(config: TAINougatConfig) -> None:
#     """Converts a PDF file to MMD format using TAI Nougat.
#     """
#     logging.info(f"Initialized NougatConfig. Config: {config}")
#
#     try:
#         logging.info("Executing conversion process.")
#         run_nougat(config)
#         logging.info(
#             "Conversion completed successfully",
#             extra={"input_pdf_paths": config.pdf_paths, "output_dir_path": str(config.output_dir)}
#         )
#     except Exception as e:
#         logging.error(
#             "Conversion failed",
#             exc_info=True,
#             extra={"input_pdf_paths": config.pdf_paths, "output_dir_path": str(config.output_dir), "error": str(e)}
#         )
#         raise
