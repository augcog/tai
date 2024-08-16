from rag.scraper.Scraper_master.base_scraper import BaseScraper
import requests
import fitz


class ScrapePdf(BaseScraper):
    """
    A class for scraping PDF files from the web, extending the functionality of BaseScraper.
    """

    def __init__(self, url):
        super().__init__(url)

    def content_extract(self, filename, url, **kwargs):
        """
        Downloads and saves a PDF from the initialized URL.
        """
        response = requests.get(self.url, stream=True)
        if response.status_code == 200:
            with open(filename, 'wb') as f:
                f.write(response.content)
            print(f"Download completed successfully and saved as {filename}")
        else:
            print(f"Failed to download the PDF. Status code: {response.status_code}")
    def add_urls_to_pdf(self, pdf_file, base_url, txt_file):
        """
        Adds a unique URL to each page of the PDF, modifies the file in place,
        and saves each URL to a .txt file.
        """
        doc = fitz.open(pdf_file)
        with open(txt_file, 'w') as f:
            for i in range(len(doc)):
                page = doc[i]
                page_url = f"{base_url}#page={i + 1}"  # Creating a unique URL for each page
                # Save the URL to the txt file
                f.write(f"Page {i + 1}: {page_url}\n")
                # Define where to place the link on the page (e.g., at the bottom of the page)
                rect = fitz.Rect(0, page.rect.height - 20, page.rect.width, page.rect.height)
                # Add the hyperlink
                page.insert_link({"kind": fitz.LINK_URI, "from": rect, "uri": page_url})
        
        doc.save(pdf_file, incremental=True, encryption = 0)  # Save changes incrementally to the original file
        doc.close() 
        print(f"URLs added to PDF and saved in place as {pdf_file}")
        print(f"Links saved to {txt_file}")


# Example usage:
if __name__ == "__main__":
    pdf_url = "https://ucb-ee106.github.io/106b-sp23site/assets/lec/lec2_quadrotor_dynamics.pdf"  # Replace with the actual PDF URL
    txt_filename = "links.txt"  # File to save the links

    pdf_saver = ScrapePdf(pdf_url)
    pdf_saver.content_extract("filename.pdf", pdf_url) # Change filename to save as and start the download process
    pdf_saver.add_urls_to_pdf("filename", pdf_url, txt_filename)