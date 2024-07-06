import os
import re
import fitz  
from pix2text import Pix2Text


def convert_pdf_to_markdown(pdf_file_path, output_file_path, page_numbers=None):
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

def extract_and_convert_pdf_to_md(pdf_path, md_path, output_folder):
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
        convert_pdf_to_markdown(single_page_pdf_path, single_page_output_folder)
        
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

def missing_page_fill(pdf_file_path, folder_name, md_file_path):
    # Ensure the output folder exists
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    
    # Extract missing pages and update Markdown content
    extract_and_convert_pdf_to_md(pdf_file_path, md_file_path, folder_name)

def generate_mmd_file_path(folder_path):
    folder_name = os.path.basename(folder_path)
    mmd_file_path = os.path.join(folder_path, f"{folder_name}_remove_image.mmd")
    return mmd_file_path
