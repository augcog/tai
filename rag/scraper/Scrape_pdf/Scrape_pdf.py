from rag.scraper.Scrape_pdf.nougat import pdf_to_md
import fitz
from pathlib import Path
from rag.scraper.Scrape_pdf.pdf_helper import missing_page_fill
import os

def create_folder_from_filename(filename: str) -> str:
    """Create a folder using the given filename.
    
    Args:
        filename (str): The filename to be used for creating the folder.
    
    Returns:
        str: The path to the created or existing folder.
    """
    # Extract the folder name from the filename (remove extension if any)
    folder_name = os.path.splitext(filename)[0]
    
    # Ensure the folder exists
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
        print(f"Created folder: {folder_name}")
    else:
        print(f"Folder already exists: {folder_name}")
    
    return folder_name

def add_remove_image_suffix(pdf_path: str) -> str:
    """Add '_remove_image' suffix to a PDF file path while keeping the .pdf extension."""
    input_path = Path(pdf_path)
    if input_path.suffix.lower() != '.pdf':
        raise ValueError("The input file must be a PDF.")
    
    # Add the suffix before the file extension
    new_path = input_path.with_stem(input_path.stem)
    
    return str(new_path)

def remove_images(input_pdf, output_pdf):
    doc = fitz.open(input_pdf)

    # Iterate through the pages
    for page_num in range(len(doc)):
        page = doc.load_page(page_num)
        images = page.get_images(full=True)
        
        # Remove each image
        for img_index in range(len(images) - 1, -1, -1):
            xref = images[img_index][0]
            page.delete_image(xref)
            
        # Optionally clean up empty spaces
        page.clean_contents()

    # Save the modified PDF
    doc.save(output_pdf)
    doc.close()
 
def process_pdf(input_pdf: str, output_folder: str) -> str:
    """Process the PDF by removing images and saving to a new file in the specified folder."""
    # Ensure the output folder exists
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    output_pdf_name = add_remove_image_suffix(os.path.basename(input_pdf))
    output_pdf = os.path.join(output_folder, output_pdf_name)
    remove_images(input_pdf, output_pdf)
    return output_pdf
    
def generate_mmd_file_path(folder_path):
    folder_name = os.path.basename(folder_path)
    mmd_file_path = os.path.join(folder_path, f"{folder_name}.mmd")
    return mmd_file_path


input_pdf = "rag\scraper\Scrape_pdf\example_pdf.pdf"



if __name__ == '__main__':
    folder_name = create_folder_from_filename(input_pdf)
    output_pdf = process_pdf(input_pdf, folder_name)
    pdf_to_md(output_pdf, folder_name)
    md_file_path = generate_mmd_file_path(folder_name)
    missing_page_fill(output_pdf, folder_name, md_file_path)
