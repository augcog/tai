import os
import re
import subprocess
from pathlib import Path

import fitz
from pix2text import Pix2Text

from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.services.tai_nougat_service import TAINougatConfig
from rag.file_conversion_router.services.tai_nougat_service.api import convert_pdf_to_mmd


class PdfConverter(BaseConverter):
    def __init__(self):
        super().__init__()
        self.available_tools = ["nougat", "MinerU"]

    def is_tool_supported(self, tool_name):
        """
        Check if a tool is supported.
        """
        return tool_name in self.available_tools

    def validate_tool(self, tool_name):
        """
        Validate if the tool is supported, raise an error if not.
        """
        if not self.is_tool_supported(tool_name):
            raise ValueError(f"Tool '{tool_name}' is not supported. Available tools: {', '.join(self.available_tools)}")

    def _to_markdown_use_MinerU(self, pdf_file_path, output_file_path):
        output_dir = os.path.dirname(output_file_path)
        os.makedirs(output_dir, exist_ok=True)

        # Construct the magic-pdf command
        command = ["magic-pdf",
                   "-p", str(pdf_file_path),
                   "-o", str(output_file_path),
                   "-m", "auto"]

        try:
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Processed: {pdf_file_path}")
            print(f"stdout: {result.stdout}")
        except subprocess.CalledProcessError as e:
            print(f"Error while processing {pdf_file_path}: {e}")
            print(f"stderr: {e.stderr}")

    def convert_pdf_to_markdown(self, pdf_file_path, output_file_path, page_numbers=None):
        """
        Convert a PDF file to Markdown format.

        Parameters:
        pdf_file_path (str): The file path of the input PDF.
        output_file_path (str): The file path where the output Markdown will be saved.
        page_numbers (list of int, optional): List of page numbers to process. Defaults to None (process all pages).
        """
        try:
            # Initialize Pix2Text with default configuration
            p2t = Pix2Text.from_config()

            # Recognize text in the PDF
            doc = p2t.recognize_pdf(pdf_file_path, page_numbers=page_numbers)

            # Save the recognized text to a Markdown file
            doc.to_markdown(output_file_path)

            print(f"Markdown saved to {output_file_path}")
        except Exception as e:
            print(f"An error occurred: {e}")

    def remove_images_from_pdf(self, input_path: Path, output_path: Path):
        pdf_document = fitz.open(input_path)

        for page_num in range(len(pdf_document)):
            page = pdf_document.load_page(page_num)
            images = page.get_images(full=True)

            # Remove each image
            for img_index in range(len(images) - 1, -1, -1):
                xref = images[img_index][0]
                page.delete_image(xref)

            # Optionally clean up empty spaces
            page.clean_contents()
        pdf_document.save(output_path)
        pdf_document.close()

    def extract_and_convert_pdf_to_md(self, pdf_path, md_path, output_folder):
        # Open the PDF document
        pdf_document = fitz.open(pdf_path)

        # Check if the Markdown file exists
        if not os.path.exists(md_path):
            print(f"Markdown file does not exist: {md_path}")
            return

        # Read the existing Markdown content
        with open(md_path, 'r', encoding='utf-8') as md_file:
            markdown_content = md_file.read()

        # Match all forms of MISSING_PAGE markers
        missing_pages = re.findall(r'\[MISSING_PAGE.*?:(\d+)\]', markdown_content)
        if missing_pages:
            self._content_logger.warning(f"Found {len(missing_pages)} Missing pages when converting {pdf_path}")

        # Extract missing pages as separate PDF files
        for page_number in missing_pages:
            page_index = int(page_number) - 1
            page = pdf_document.load_page(page_index)
            single_page_pdf_path = os.path.join(output_folder, f"page_{page_number}.pdf")
            single_page_document = fitz.open()
            single_page_document.insert_pdf(pdf_document, from_page=page_index, to_page=page_index)
            single_page_document.save(single_page_pdf_path)

            # Run Nougat on the single page PDF
            single_page_output_folder = os.path.join(output_folder, f"page_{page_number}_output")
            if not os.path.exists(single_page_output_folder):
                os.makedirs(single_page_output_folder)
            self.convert_pdf_to_markdown(single_page_pdf_path, single_page_output_folder)

            # Read the generated Markdown content for this page
            single_page_md_files = os.listdir(single_page_output_folder)
            if not single_page_md_files:
                print(f"No Markdown file generated for page {page_number}")
                continue

            single_page_md_path = os.path.join(single_page_output_folder, single_page_md_files[0])
            with open(single_page_md_path, 'r', encoding='utf-8') as single_page_md_file:
                single_page_md_content = single_page_md_file.read()

            # Escape backslashes in single_page_md_content
            single_page_md_content = single_page_md_content.replace('\\', '\\\\')

            self._content_logger.info(f"missing page {page_number} replaced"
                                      f"with content: {single_page_md_content[:100]}...")

            # Replace the missing page marker with the actual content
            markdown_content = re.sub(rf'\[MISSING_PAGE.*?:{page_number}\]', single_page_md_content, markdown_content)

        pdf_document.close()

        # Save the updated Markdown content
        with open(md_path, 'w', encoding='utf-8') as md_file:
            md_file.write(markdown_content)

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path, conversion_method: str = "nougat") -> Path:
        # """Perform PDF to Markdown conversion using Nougat with the detected hardware configuration."""
        self.validate_tool(conversion_method)
        temp_dir_path = output_path.parent

        # Create the directory if it doesn't exist
        if not temp_dir_path.exists():
            os.makedirs(temp_dir_path)

        # Convert the PDF to Markdown using Nougat.
        if conversion_method == "nougat":
            print('Using Nougat')
            # Define the path for the PDF without images in the output directory
            pdf_without_images_path = temp_dir_path / input_path.name
            # Remove images from the PDF and save to the output directory
            self.remove_images_from_pdf(input_path, pdf_without_images_path)
            self._to_markdown_using_tai_nougat(pdf_without_images_path, output_path)
            # Now change the file name of generated mmd file to align with the expected md file path from base converter
            output_mmd_path = output_path.with_suffix(".mmd")
            self.extract_and_convert_pdf_to_md(str(pdf_without_images_path), str(output_mmd_path), str(temp_dir_path))
            target = output_path.with_suffix(".md")
            output_mmd_path.rename(target)
            print(output_mmd_path)
            # Rename it to md file
            target = output_path.with_suffix(".md")


        elif conversion_method == "MinerU":
            print("Using MinerU")
            self._to_markdown_use_MinerU(input_path, temp_dir_path)
            # Construct the correct path based on the nested folder structure
            base_name = input_path.stem  # e.g., "61a-sp24-mt1"
            md_file_path = temp_dir_path / base_name / "auto" / f"{base_name}.md"
            if md_file_path.exists():
                print(f"Markdown file found: {md_file_path}")
            else:
                raise FileNotFoundError(f"Markdown file not found: {md_file_path}")
            # Set the target to this markdown path
            target = md_file_path
        return target

    def _to_markdown_using_native_nougat_cli(self, input_pdf_path: Path, output_path: Path) -> None:
        """
        Perform PDF to Markdown conversion using Native Nougat CLI.

        The native nougat cli is in the predict.py from meta nougat repo.
        Parameters except input and output path are hard coded for now.
        """
        default_nougat_config = TAINougatConfig()
        command = [
            "nougat",
            str(input_pdf_path),
            # nougat requires the argument output path to be a directory, not file, so we need to handle it here
            "-o",
            str(output_path.parent),
            "--no-skipping" if not default_nougat_config.skipping else "",
            "--recompute" if default_nougat_config.recompute else "",
            "--model",
            default_nougat_config.model_tag,
        ]
        command = [str(arg) for arg in command]
        try:
            result = subprocess.run(command, check=False, capture_output=True, text=True)
            self._logger.info(f"Output: {result.stdout}")
            self._logger.info(f"Errors: {result.stderr}")
            if result.returncode != 0:
                self._logger.error(f"Command exited with a non-zero status: {result.returncode}")
        except Exception as e:
            self._logger.error(f"An error occurred: {str(e)}")
            raise

    @staticmethod
    def _to_markdown_using_tai_nougat(input_pdf_path: Path, output_path: Path) -> None:
        """Perform PDF to Markdown conversion using TAI Nougat.

        TAI nougat is our custom implementation of the Nougat API, with better performance and abstraction.
        """
        config = TAINougatConfig(
            pdf_paths=[input_pdf_path],
            output_dir=output_path.parent,
        )
        convert_pdf_to_mmd(config)