# from .config_nougat.tai_nougat_config import TAINougatConfig
# from .mlx_nougat_service import api as mlx_nougat_api
# from .torch_nougat_service import api as torch_nougat_api
#
#
# def run_nougat(config: TAINougatConfig):
#     """Run Nougat with the provided configuration.
#     model is initialized using config only on the first time run_nougat is called .
#     """
#     input_path = config.pdf_paths[0]
#     output_dir = config.output_dir
#
#     using_torch = config.using_torch
#
#     if using_torch:
#         torch_nougat_api.convert_pdf_to_mmd(input_path, output_dir)
#     else:
#         mlx_nougat_api.convert_pdf_to_mmd(input_path, output_dir)
