import re
from pathlib import Path
from urllib.parse import urlparse

from bs4 import BeautifulSoup
from markdownify import markdownify as md

from file_conversion_router.conversion.base_converter import BaseConverter

content_tags_dict = {
    "https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html": [
        ("div", {"class": "contents"})
    ],
    "https://www.berkeley.edu/": [],
    "https://wiki.ros.org/ROS/Tutorials/": [
        ("div", {"id": "page", "lang": "en", "dir": "ltr"})
    ],
    "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/": [
        ("div", {"class": "archive", "id": "archive"})
    ],
    "https://platform.openai.com/docs/introduction": [("div", {"class": "docs-body"})],
    "https://cloud.google.com/docs/": [
        ("main", {"role": "main", "class": "devsite-main-content"})
    ],
    "https://docs.qualcomm.com/bundle/publicresource/topics/80-62010-1/Overview.html?product=Windows%20on%20Snapdragon": [
        (
            "div",
            {
                "text": "This section defines contents of documents",
                "class": "contentmodule ng-star-inserted",
            },
        )
    ],
    "https://www.svlsimulator.com/docs/": [("div", {"role": "main"})],
    "https://cs61a.org/": [
        ("section", {"id": "calendar", "class": "table", "cellpadding": "5px"}),
        ("div", {"class": "col-md-9"}),
    ],
    "https://ucb-ee106.github.io/106b-sp23site/": [("main", {"class": "main-content"})],
    "https://en.wikipedia.org/wiki/University_of_California,_Berkeley": [
        ("main", {"id": "content", "class": "mw-body"})
    ],
    "https://freesociologybooks.com/Sociology_Of_The_Family/01_Changes_and_Definitions.php#google_vignette": [
        ("div", {"id": "content", "class": "clearfix"})
    ],
    "https://leginfo.legislature.ca.gov/faces/codes.xhtml": [
        ("div", {"class": "tab_content"})
    ],
    "https://guide.berkeley.edu/courses/": [],
}


class HtmlConverter(BaseConverter):
    def __init__(self, course_name: str, course_id: str):
        super().__init__(course_name, course_id)
        self.index_helper = [dict]

    # Override
    def _to_markdown(self, input_path: Path, output_path: Path) -> Path:
        def get_content_tags(url, content_tags_dict):
            parsed_url = urlparse(url)

            for key in content_tags_dict:
                parsed_key = urlparse(key)

                # Check if the base URL matches and the path is a subpath
                if (
                    parsed_url.netloc == parsed_key.netloc
                    and parsed_url.path.startswith(parsed_key.path)
                ):
                    return content_tags_dict[key]

            return None  # Return None if no match is found

        output_path = output_path.with_suffix(".md")
        with open(input_path, "r") as input_file:
            html_content = input_file.read()
        stem = input_path.stem
        metadata_path = input_path.with_name(f"{input_path.stem}_metadata.yaml")
        metadata_content = self._read_metadata(metadata_path)
        url = metadata_content.get("URL")

        # Parse the HTML content
        soup = BeautifulSoup(html_content, "html.parser")
        content_tags = get_content_tags(url, content_tags_dict)
        markdown_outputs = []
        if content_tags:
            for tag_type, tag_attr in content_tags:
                # Find the specific tags
                content = soup.find_all(tag_type, tag_attr)
                for item in content:
                    # Convert each HTML item to Markdown
                    markdown = md(str(item), heading_style="ATX", default_title=True)
                    modified_content = re.sub(
                        r"(?<!^)(```)", r"\n\1", markdown, flags=re.MULTILINE
                    )
                    markdown_outputs.append(modified_content)
            # Concatenate all markdown outputs with a newline
            final_markdown = "\n\n".join(markdown_outputs)
        else:
            final_markdown = md(str(soup), heading_style="ATX", default_title=True)
            final_markdown = re.sub(
                r"(?<!^)(```)", r"\n\1", final_markdown, flags=re.MULTILINE
            )
        with open(output_path, "w") as output_file:
            output_file.write(final_markdown)

        return output_path
