import subprocess
from pathlib import Path
import os
import fitz
import re
from pix2text import Pix2Text


from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.utils.hardware_detection import detect_gpu_setup
from rag.file_conversion_router.classes.page import Page
from rag.file_conversion_router.classes.chunk import Chunk
import yaml


class PdfConverter(BaseConverter):
    def __init__(self, model_tag: str = "0.1.0-small", batch_size: int = 4):
        super().__init__()
        self.model_tag = model_tag
        self.batch_size = batch_size
        self.device_type, _ = detect_gpu_setup()  # Detect hardware configuration

        self._validate_parameters()

        self._logger.info(f"Using {self.device_type} on Torch")

    def _validate_parameters(self):
        """Validate model tag and batch size."""
        if not isinstance(self.batch_size, int) or self.batch_size < 0:
            raise ValueError("Batch size must be a non-negative integer.")
        # https://github.com/facebookresearch/nougat/issues/179#issuecomment-1831849650
        if self.device_type == "cpu":
            self.batch_size = 0
            self._logger.info("Forcing batch size to 0 for running on CPU")
        acceptable_models = ["0.1.0-small", "0.1.0-base"]
        if self.model_tag not in acceptable_models:
            raise ValueError(f"Model tag must be one of {acceptable_models}")
        
    def convert_pdf_to_markdown(self, pdf_file_path, output_file_path, page_numbers=None):
    # """
    # Convert a PDF file to Markdown format.

    # Parameters:
    # pdf_file_path (str): The file path of the input PDF.
    # output_file_path (str): The file path where the output Markdown will be saved.
    # page_numbers (list of int, optional): List of page numbers to process. Defaults to None (process all pages).
    # """
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

            # Replace the missing page marker with the actual content
            markdown_content = re.sub(rf'\[MISSING_PAGE.*?:{page_number}\]', single_page_md_content, markdown_content)
        
        pdf_document.close()
        
        # Save the updated Markdown content
        with open(md_path, 'w', encoding='utf-8') as md_file:
            md_file.write(markdown_content)

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        # """Perform PDF to Markdown conversion using Nougat with the detected hardware configuration."""
        temp_dir_path = output_path.parent
        
        # Create the directory if it doesn't exist
        if not temp_dir_path.exists():
            os.makedirs(temp_dir_path)
        
        # Define the path for the PDF without images in the output directory
        pdf_without_images_path = temp_dir_path / input_path.name
        
        # Remove images from the PDF and save to the output directory
        self.remove_images_from_pdf(input_path, pdf_without_images_path)

        command = [
            "nougat",
            str(pdf_without_images_path),
            # nougat requires the argument output path to be a directory, not file, so we need to handle it here
            "-o",
            str(output_path.parent),
            "--no-skipping",
            "--model",
            self.model_tag,
            # "--batchsize",
            # str(self.batch_size),
        ]
        try:
            result = subprocess.run(command, check=False, capture_output=True, text=True)
            self._logger.info(f"Output: {result.stdout}")
            self._logger.info(f"Errors: {result.stderr}")
            if result.returncode != 0:
                self._logger.error(f"Command exited with a non-zero status: {result.returncode}")
        except Exception as e:
            self._logger.error(f"An error occurred: {str(e)}")
            raise
        
        # Now change the file name of generated mmd file to align with the expected md file path from base converter
        output_mmd_path = output_path.with_suffix(".mmd")
        self.extract_and_convert_pdf_to_md(str(pdf_without_images_path), str(output_mmd_path), str(temp_dir_path))
        # Rename it to `md` file
        target = output_path.with_suffix(".md")
        output_mmd_path.rename(target)
        print(output_mmd_path)
        return target
        #
        # except Exception as e:
        #     self._logger.error(f"An error occurred {str(e)})")
        #     raise

    # def _to_page(self, input_path: Path, output_path: Path) -> Page:
    #     """Perform Markdown to Page conversion."""
    #     md_file = self._to_markdown(input_path, output_path,)
    #     # except Exception as e:
    #         # self._logger.error(f"An error occurred during markdown conversion: {str(e)}")
    #         # raise

    #     output_path.parent.mkdir(parents = True, exist_ok = True)

    #     filetype = md_file.suffix.lstrip('.')
    #     with open(md_file, "r") as input_file:
    #         text = input_file.read()


    #     metadata_path = input_path.with_name(f"{input_path.stem}_metadata.yaml")
    #     metadata_content = self._read_metadata(metadata_path)
    #     url = metadata_content.get("URL")
    #     return Page(pagename = input_path.stem, content={'text': text}, filetype = filetype, page_url=url)