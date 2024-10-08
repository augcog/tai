from rag.scraper.Scraper_master.base_scraper import BaseScraper
import requests
import fitz
import yaml


class ScrapePdf(BaseScraper):
    """
    A class for scraping PDF files from the web, extending the functionality of BaseScraper.
    """

    def __init__(self, url):
        super().__init__(url)

    def content_extract(self, filename, url, **kwargs):
        """
        Downloads and saves a PDF from the initialized URL and stores the URL in a metadata file.
        """
        response = requests.get(url, stream=True)
        if response.status_code == 200:
            with open(filename, 'wb') as f:
                f.write(response.content)
            print(f"Download completed successfully and saved as {filename}")
            
            metadata_filename = filename.replace(".pdf", "_metadata.yaml")

            metadata_content = {
                "URL": url
            }
            self.add_urls_to_pdf(filename, url)
            with open(metadata_filename, 'w', encoding='utf-8') as metafile:
                yaml.dump(metadata_content, metafile)
            
            print(f"Metadata saved successfully as {metadata_filename}")
        else:
            print(f"Failed to download the PDF. Status code: {response.status_code}")
            
    def add_urls_to_pdf(self, pdf_file, pdf_url):
        """
        Adds a unique URL to each page of the PDF, modifies the file in place,
        and saves each URL to a .txt file.
        """
        doc = fitz.open(pdf_file)
        for i in range(len(doc)):
            page = doc[i]
            page_url = f"{pdf_url}#page={i + 1}"  # Creating a unique URL for each page
            # Define where to place the link on the page (e.g., at the bottom of the page)
            rect = fitz.Rect(0, page.rect.height - 20, page.rect.width, page.rect.height)
            # Add the hyperlink
            page.insert_link({"kind": fitz.LINK_URI, "from": rect, "uri": page_url})
        
        doc.save(pdf_file, incremental=True, encryption = 0)  # Save changes incrementally to the original file
        doc.close() 
        print(f"URLs added to PDF and saved in place as {pdf_file}")
        


# Example usage:
if __name__ == "__main__":
    pdf_url = "https://ucb-ee106.github.io/106b-sp23site/assets/disc/Discussion_1_Dynamical_Systems_Solution.pdf"  # Replace with the actual PDF URL
    txt_filename = "links.txt"  # File to save the links

    pdf_saver = ScrapePdf(pdf_url)
    pdf_saver.content_extract("filename.pdf", pdf_url) # Change filename to save as and start the download process
    pdf_saver.add_urls_to_pdf("filename.pdf", pdf_url)
