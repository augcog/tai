from file_conversion_router.conversion.base_converter import BaseConverter
from file_conversion_router.services.tai_MinerU_service.api import convert_pdf_to_md_by_MinerU
from pathlib import Path
import json
import re


class PdfConverter(BaseConverter):
    def __init__(self, course_name, course_id):
        super().__init__(course_name, course_id)
        self.available_tools = ["MinerU"]
        self.index_helper = None
        self.file_name = ""


    def remove_image_links(self, text):
        """
        Remove image links from the text.
        """
        # Regular expression to match image links
        image_link_pattern = r"!\[.*?\]\(.*?\)"
        # Remove all image links
        return re.sub(image_link_pattern, "", text)

    def clean_markdown_content(self, markdown_path):
        with open(markdown_path, "r", encoding="utf-8") as file:
            content = file.read()
        # Remove image links (assuming this method exists)
        cleaned_content = self.remove_image_links(content)
        # Remove lines that contain only hash symbols and spaces
        # This pattern matches lines with only #, spaces, and optionally newlines
        cleaned_content = re.sub(r'^[ #]+$', '-------------', cleaned_content, flags=re.MULTILINE)
        with open(markdown_path, "w", encoding="utf-8") as file:
            file.write(cleaned_content)
        return cleaned_content

    # Override
    def _to_markdown(
        self, input_path: Path, output_path: Path, conversion_method: str = "MinerU"
    ) -> Path:
        self.file_name = input_path.name
        output_dir = output_path.parent

        if conversion_method == "MinerU":
            md_file_path = convert_pdf_to_md_by_MinerU(input_path, output_dir)

            if md_file_path.exists():
                print(f"Markdown file found: {md_file_path}")
            else:
                raise FileNotFoundError(f"Markdown file not found: {md_file_path}")
            # Set the target to this markdown path
            target = md_file_path
            cleaned_content = self.clean_markdown_content(target)
            json_file_path = md_file_path.with_name(f"{md_file_path.stem}_content_list.json")
            with open(json_file_path, "r", encoding="utf-8") as f_json:
                data = json.load(f_json)
            self.generate_index_helper(data, md=cleaned_content)
            return target

    def generate_index_helper(self, data, md=None):
        self.index_helper = []
        for item in data:
            if item.get('text_level') == 1:
                title = item['text'].strip()
                if title.startswith('# '):
                    title = title[2:]

                skip_patterns = [
                    re.compile(r'^\s*ROAR ACADEMY EXERCISES\s*$', re.I),
                    re.compile(r'^\s*(?:#+\s*)+$')  # lines that are only # + spaces
                ]
                if any(p.match(title) for p in skip_patterns):
                    continue

                # Check if title appears after any number of # symbols
                if md:
                    lines = md.split('\n')
                    title_found = False
                    for line in lines:
                        stripped_line = line.strip()
                        if stripped_line.startswith('#') and title in stripped_line:
                            # More precise check: extract the heading text
                            heading_text = re.sub(r'^#+\s*', '', stripped_line).strip()
                            if heading_text == title:
                                title_found = True
                                break

                    if title_found:
                        page_index = item['page_idx'] + 1  # 1-based
                        self.index_helper.append({title: page_index})