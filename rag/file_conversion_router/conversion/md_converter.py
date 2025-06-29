from pathlib import Path
import yaml
from rag.file_conversion_router.conversion.base_converter import BaseConverter
from rag.file_conversion_router.classes.page import Page
from rag.file_conversion_router.classes.chunk import Chunk


class MarkdownConverter(BaseConverter):
    def __init__(self):
        super().__init__()

    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        """Perform reStructuredText to Markdown conversion."""
        output_path = output_path.with_suffix(".md")
        try:
            with open(input_path, "r", encoding="utf-8") as input_file:
                content = input_file.read()
                if not content:
                    self._logger.warning(f"Input file {input_path} is empty.")
                with open(output_path, "w", encoding="utf-8") as output_file:
                    output_file.write(content)
        except Exception as e:
            self._logger.error(f"Error reading file {input_path}: {str(e)}")
        return output_path

    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        """Perform Markdown to Page conversion."""
        try:
            self._to_markdown(input_path, output_path)
        except Exception as e:
            self._logger.error(
                f"An error occurred during markdown conversion: {str(e)}"
            )
            raise

        output_path.parent.mkdir(parents=True, exist_ok=True)

        filetype = input_path.suffix.lstrip(".")
        with open(input_path, "r", encoding="utf-8") as input_file:
            text = input_file.read()

        metadata_path = input_path.with_name(f"{input_path.stem}_metadata.yaml")
        metadata_content = self._read_metadata(metadata_path)
        url = metadata_content.get("URL")
        return Page(
            pagename=input_path.stem,
            content={"text": text},
            filetype=filetype,
            page_url=url,
        )


# if __name__ == "__main__":
#     converter = MarkdownConverter()
# # Run the conversion to Page
# page = converter._to_page(Path("tests\\test_rag\data\integrated_tests\input_folder2_nested_folder_pdf+md\mds\section-3-API-based-scraping.md"), Path("tests\\test_rag\data\integrated_tests\expected_output_folder2_nested_folder_pdf+md\section-3-API-based-scraping\section-3-API-based-scraping.md"))
