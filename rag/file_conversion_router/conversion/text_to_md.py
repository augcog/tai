# # file_conversion_router/conversion/text_to_md.py
# from .base_converter import BaseConverter
# import logging
#
#
# class TextToMdConverter(BaseConverter):
#     def convert(self, input_path: str, output_path: str):
#         logging.info(f"Starting text to Markdown conversion for {input_path}")
#         # Directly copying text for simplicity; enhance as needed
#         with open(input_path, 'r') as input_file, open(output_path, 'w') as output_file:
#             for line in input_file:
#                 output_file.write(line)
#         logging.info(f"Text conversion completed for {input_path}")
