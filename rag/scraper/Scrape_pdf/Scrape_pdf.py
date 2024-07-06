from nougat import pdf_to_md
import fitz
from pathlib import Path
from pdf_helper import missing_page_fill
import os


def add_remove_image_suffix(pdf_path: str) -> str:
    """Add '_remove_image' suffix to a PDF file path while keeping the .pdf extension."""
    input_path = Path(pdf_path)
    if input_path.suffix.lower() != '.pdf':
        raise ValueError("The input file must be a PDF.")
    
    # Add the suffix before the file extension
    new_path = input_path.with_stem(input_path.stem + "_remove_image")
    
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
 
def process_pdf(input_pdf: str) -> str:
    """Process the PDF by removing images and saving to a new file with '_remove_image' suffix."""
    output_pdf = add_remove_image_suffix(input_pdf)
    remove_images(input_pdf, output_pdf)
    return output_pdf
    
def generate_mmd_file_path(folder_path):
    folder_name = os.path.basename(folder_path)
    mmd_file_path = os.path.join(folder_path, f"{folder_name}_remove_image.mmd")
    return mmd_file_path


input_pdf = "rag\scraper\Scrape_pdf\example_pdf.pdf"
folder_name = "rag\scraper\Scrape_pdf\example_pdf"


if __name__ == '__main__':
    output_pdf = process_pdf(input_pdf)
    pdf_to_md(output_pdf, folder_name)
    md_file_path = generate_mmd_file_path(folder_name)
    missing_page_fill(output_pdf, folder_name, md_file_path)
