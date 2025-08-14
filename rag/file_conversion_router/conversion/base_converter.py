"""Base classes for all file type converters."""
import string
import logging
from abc import ABC, abstractmethod
# from concurrent.futures import Future, ThreadPoolExecutor
from shutil import copy2
# from threading import Lock
from typing import Dict, List, Union, Tuple, Any

from file_conversion_router.classes.chunk import Chunk
from file_conversion_router.classes.new_page import Page
from file_conversion_router.embedding_optimization.src.pipeline.optimizer import (
    EmbeddingOptimizer,
)
from file_conversion_router.utils.logger import (
    conversion_logger,
    logger,
    content_logger,
)
from file_conversion_router.utils.utils import ensure_path
from file_conversion_router.utils.title_handle import *


class BaseConverter(ABC):
    """Base classes for all file type converters.

    This base classes defines the interface for all type converters, with a standardized workflow, which outputs 3 files:
    - Markdown
    - Tree txt
    - Pickle

    All child classes need to implement the abstract methods:
    - _to_markdown

    As long as a child classes can convert a file to Markdown,
    the base classes will handle the rest of the conversion process.
    """

    DEFAULT_EMBEDDING_OPTIMIZATION_CONFIG_PATH = str(
        (
            Path(__file__).parent
            / ".."
            / "embedding_optimization"
            / "src"
            / "configs"
            / "default_config.yaml"
        ).resolve()
    )

    def __init__(
        self, course_name, course_id, file_uuid, optimizer_config_path: Union[str, Path] = None
    ):
        self.index_helper = None
        self.course_name = course_name
        self.course_id = course_id
        self._md_parser = None

        self._md_path = None
        self._logger = logger
        self._content_logger = content_logger
        self.file_name = ""
        self.file_uuid = file_uuid
        # if optimizer_config_path is None:
        #     optimizer_config_path = self.DEFAULT_EMBEDDING_OPTIMIZATION_CONFIG_PATH
        # self.optimizer_config_path = optimizer_config_path

        if optimizer_config_path:
            config_path = Path(optimizer_config_path)
            if not config_path.is_file():
                self._logger.error(
                    f"Optimizer config file does not exist at: {config_path}"
                )
                raise FileNotFoundError(
                    f"Optimizer config file does not exist at: {config_path}"
                )

            self.optimizer = EmbeddingOptimizer(config_path=str(config_path))
            self._logger.info(
                f"EmbeddingOptimizer initialized with config: {config_path}"
            )
        else:
            self.optimizer = None
            self._logger.info("Embedding optimization is disabled.")

    @conversion_logger
    def convert(
        self, input_path: Union[str, Path], output_folder: Union[str, Path], input_root: Union[str, Path] = None,
    ) -> Tuple[List[Chunk], dict]:
        """Convert an input file to mark down under the output folder.

        Args:
            input_path: The path for a single file to be converted. e.g. 'path/to/file.txt'
            output_folder: The folder where the output files will be saved. e.g. 'path/to/output_folder'
                other files will be saved in the output folder, e.g.:
                - 'path/to/output_folder/file.md'
            input_root: The root folder of the input file, used to calculate the relative path of the input file.
        """
        self.file_name = input_path.name
        self.relative_path = input_path.relative_to(input_root)
        input_path, output_folder = ensure_path(input_path), ensure_path(output_folder)
        if not input_path.exists():
            self._logger.error(f"The file {input_path} does not exist.")
            raise FileNotFoundError(f"The file {input_path} does not exist.")
        chunks, metadata = self._convert(input_path, output_folder)
        # future = self.cache.get_future(file_hash)
        # if not future or not future.running():
        #     with ThreadPoolExecutor() as executor:
        #         future = executor.submit(
        #             self._convert_and_cache, input_path, output_folder, file_hash
        #         )
        #         self.cache.store_future(file_hash, future)
        #         self._logger.info(
        #             "New conversion task started or previous task completed."
        #         )
        #         return
        #
        # self._logger.info(
        #     "Conversion is already in progress, waiting for it to complete."
        # )
        # # This will block until the future is completed
        # future.result()
        return chunks, metadata

    @conversion_logger
    def _convert_to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format."""
        self._to_markdown(input_path, output_path)

    def _setup_output_paths(
        self, input_path: Union[str, Path], output_folder: Union[str, Path]
    ) -> None:
        """Set up the output paths for the Markdown, tree txt, and pkl files."""
        input_path = ensure_path(input_path)
        output_folder = ensure_path(output_folder)
        self.file_name = input_path.name
        self._md_path = ensure_path(output_folder / f"{self.file_name}.md")

    def _convert(self, input_path: Path, output_folder: Path,) -> Tuple[List[Chunk], dict]:
        """Convert the input file to Mark down and return the chunks."""
        self._setup_output_paths(input_path, output_folder)
        (chunks, metadata), conversion_time = self._perform_conversion(input_path, output_folder)
        return chunks, metadata

    def _read_metadata(self, metadata_path: Path) -> dict:
        """Read metadata from file or return mocked data if file doesn't exist."""
        if metadata_path.exists():
            try:
                with open(metadata_path, "r") as metadata_file:
                    return yaml.safe_load(metadata_file)
            except Exception as e:
                self._logger.error(f"Error reading metadata file: {str(e)}")
                return {"URL": "", }
        else:
            self._logger.warning(
                f"Metadata file not found: {metadata_path}. Using mocked metadata."
            )
            return {"URL": "", }

    @conversion_logger
    def _perform_conversion(self, input_path: Path, output_folder: Path) -> Tuple[List[Chunk], dict]:
        """Perform the file conversion process."""
        logging.getLogger().setLevel(logging.INFO)
        logger.info(f"🚀 Starting conversion for {input_path}")
        if not output_folder.exists():
            output_folder.mkdir(parents=True, exist_ok=True)
            logger.warning(
                f"Output folder did not exist, it's now created: {output_folder}"
            )
        logger.info(f"📄 Expected Markdown Path: {self._md_path}")
        page, metadata = self._to_page(input_path, self._md_path)
        logger.info(f"✅ Page conversion completed for {input_path}.")
        chunks = page.to_chunk()
        logger.info("✅ Successfully converted page content to chunks.")
        return chunks, metadata

    def _to_page(self, input_path: Path, output_path: Path) -> Tuple[Page, dict]:
        """Convert the input file to a Page object and return it along with metadata."""
        # Ensure the output directory exists
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.file_type = input_path.suffix.lstrip(".")
        md_path = self._to_markdown(input_path, output_path)
        metadata_path = input_path.with_name(f"{self.file_name}_metadata.yaml")
        if md_path:
            structured_md, content_dict = self.apply_markdown_structure(input_md_path=md_path, file_type=self.file_type)
            with open(md_path, "w", encoding="utf-8") as md_file:
                md_file.write(structured_md)
        else:
            structured_md = ""
            content_dict = {}
        metadata_content = self._read_metadata(metadata_path)
        metadata_content = {'URL': metadata_content.get('URL', '')}
        metadata = self._put_content_dict_to_metadata(
            content_dict=content_dict,
            metadata_content=metadata_content,
        )
        with open(metadata_path, "w", encoding="utf-8") as yaml_file:
            yaml.safe_dump(metadata, yaml_file, allow_unicode=True)
        url = metadata_content.get("URL")
        content = {"text": structured_md}
        return Page(
            course_name=self.course_name,
            course_id=self.course_id,
            filetype=self.file_type,
            content=content,
            page_name=self.file_name,
            page_url=url,
            index_helper=self.index_helper,
            file_path=self.relative_path,
            file_uuid=self.file_uuid,
        ), metadata

    def _put_content_dict_to_metadata(self, content_dict: dict, metadata_content: dict) -> dict:

        metadata_content['file_uuid'] = self.file_uuid
        metadata_content["file_name"] = str(self.file_name)
        metadata_content['file_path'] = str(self.relative_path)
        metadata_content["course_name"] = self.course_name
        metadata_content["course_id"] = self.course_id
        if not content_dict:
            return metadata_content
        metadata_content["sections"] = content_dict['key_concepts']
        for section in metadata_content["sections"]:
            section["name"] = section.pop('source_section_title')
            section["index"] = section.pop('source_section_index')
            section["key_concept"] = section.pop('concepts')
            section["aspects"] = section.pop('content_coverage')
            for aspect in section["aspects"]:
                aspect["type"] = aspect.pop('aspect')
        if self.file_type == "ipynb":
            metadata_content["problems"] = self.process_problems(content_dict)
        return metadata_content

    def match_a_title_and_b_title(self, a_titles: str, b_titles: str, operator):
        """
        Match the helper titles with the levels titles.
        This method is used to fix the index_helper with titles and their levels.
        """
        a_titles = a_titles.translate(str.maketrans('', '', string.punctuation)).lower().strip()
        # a_titles = a_titles.replace('"',"'").lower().strip()
        b_titles = b_titles.translate(str.maketrans('', '', string.punctuation)).lower().strip()
        # b_titles = b_titles.replace('"',"'").lower().strip()
        return operator(a_titles, b_titles)

    def process_problems(self, content_dict):
        # Return just the list of problems, not a dictionary
        problems_list = []
        for problem in content_dict['problems']:
            processed_problem = {}
            for title in self.index_helper:
                if self.match_a_title_and_b_title(title[-1],problem['ID'], str.__contains__):
                    processed_problem['problem_index'] = self.index_helper[title][0]
                    break
            else:
                processed_problem['problem_index'] = None
            processed_problem['problem_id'] = problem['ID']
            processed_problem['problem_content'] = problem['content']

            # Create questions structure
            processed_problem['questions'] = {}
            for i in range(1, 3):
                question_key = f'sub_problem_{i}'
                sub_prob=problem[question_key]
                processed_problem['questions'][f'question_{i}'] = {
                    'question': sub_prob.get('description_of_problem', ''),
                    'choices': sub_prob.get('options', []),
                    'answer': sub_prob.get('answers_options', []),
                    'explanation': sub_prob.get('explanation_of_answer', '')
                }

            problems_list.append(processed_problem)
        return problems_list

    def count_header_levels(self, content_text: str) -> int:
        """
        Count the number of unique header levels in the markdown content.
        """
        header_levels = set()
        for line in content_text.splitlines():
            if line.startswith("#"):
                level = line.count("#")
                header_levels.add(level)
        return len(header_levels)
    def update_content_dict_titles_with_levels(self, content_dict: dict,content_text: str) -> dict:
        """
        Update the content_dict with titles and their levels from the markdown content.
        """
        titles_with_levels = []
        for line in content_text.splitlines():
            if line.startswith("#"):
                level = line.count("#")
                title = line.lstrip("#").strip()
                if title == "":
                    continue
                if title.startswith('*'):
                    title = title.lstrip('*').rstrip('*').strip()
                titles_with_levels.append({"title": title, "level_of_title": level})
        content_dict["titles_with_levels"] = titles_with_levels
        return content_dict

    def fix_index_helper_with_titles_with_level(self, content_dict: dict):
        title_with_levels = content_dict.get("titles_with_levels", [])
        index_helper = []
        twl_index = 0
        for item in self.index_helper:
            if self.match_a_title_and_b_title(list(item.keys())[0], title_with_levels[twl_index]["title"], str.__eq__):
                index_helper.append(item)
                title_with_levels[twl_index]["title"]=list(item.keys())[0]
                twl_index += 1
                if twl_index >= len(title_with_levels):
                    break
        if len(index_helper) != len(title_with_levels):
            raise AssertionError(f"twl_index: {twl_index} != len(title_with_levels): {len(title_with_levels)}")
        self.index_helper = index_helper


    def apply_markdown_structure(
        self, input_md_path: Path | None, file_type: str):
        """
        Apply the markdown structure based on the file type and content.
        input_md_path: Path to the input markdown file.
        file_type: Type of the file, e.g., "mp4", "pdf", "ipynb".
        returns: md content and a dictionary with structured content.
        """
        file_name = input_md_path.stem
        with open(input_md_path, "r", encoding="UTF-8") as input_file:
            content_text = input_file.read()
            pattern = r'^\s*#\s*ROAR ACADEMY EXERCISES\s*$'
            content_text = re.sub(pattern, '', content_text, flags=re.MULTILINE)
        header_levels = self.count_header_levels(content_text)
        if header_levels == 0 and file_type == "mp4" or file_type == 'mkv' or file_type == 'webm':
            json_path = input_md_path.with_suffix(".json")
            content_dict = get_structured_content_without_title(
                md_content=content_text,
                file_name=file_name,
                course_name=self.course_name,
            )
            new_md = apply_structure_for_no_title(
                md_content=content_text, content_dict=content_dict
            )
            self.update_index_helper(content_dict,new_md)
            add_titles_to_json(index_helper=self.index_helper, json_file_path=json_path)
        elif file_type == "ipynb":
            content_dict = get_strutured_content_for_ipynb(
                md_content=content_text,
                file_name=file_name,
                course_name=self.course_name,
                index_helper=self.index_helper,
            )
            content_dict=self.update_content_dict_titles_with_levels(
                content_dict=content_dict, content_text=content_text
            )
            self.fix_index_helper_with_titles_with_level(content_dict)
            new_md =content_text

        elif file_type == "pdf":
            content_dict = get_structured_content_with_one_title_level(
                md_content=content_text,
                file_name=file_name,
                course_name=self.course_name,
                index_helper=self.index_helper,
            )
            new_md = apply_structure_for_one_title(
                md_content=content_text, content_dict=content_dict, index_helper=self.index_helper
            )
        else:
            content_dict = get_only_key_concepts(
                md_content=content_text,
                file_name=file_name,
                course_name=self.course_name,
                index_helper =self.index_helper)
            content_dict=self.update_content_dict_titles_with_levels(content_dict=content_dict, content_text=content_text)
            self.fix_index_helper_with_titles_with_level(content_dict)
            new_md=content_text

        content_dict = self.add_source_section_index(content_dict= content_dict, md_content=new_md)
        return new_md, content_dict

    def generate_index_helper(self, md: str, data = None):
        """ Generate an index helper from the Markdown content.
        """
        self.index_helper = []
        lines = md.splitlines()
        for i, line in enumerate(lines):
            if line.startswith("#"):
                title = line.strip().lstrip("#").strip()
                if title == "":
                    continue
                self.index_helper.append({title: i + 1})

    def add_source_section_index(self, content_dict: dict, md_content: str = None) -> dict:
        #If not mp4, update the index_helper with titles and their levels, else don't update it.
        if self.file_type !="mp4" and self.file_type != 'mkv' and self.file_type != 'webm':
            self.update_index_helper(content_dict, md_content=md_content)
        # If there are key concepts, update their source_section_title and source_section_index
        if 'key_concepts' in content_dict:
            for concept in content_dict['key_concepts']:
                source_title = concept['source_section_title']
                found = False
                for titles in self.index_helper.keys():
                    real_title = titles[-1] if isinstance(titles, tuple) else titles
                    if self.match_a_title_and_b_title(real_title, source_title, str.__contains__):
                        concept['source_section_title'] = real_title
                        concept['source_section_index'] = self.index_helper[titles][0] # page index
                        found = True
                        break
                if not found:
                    raise ValueError(
                        f"Source section title '{source_title}' not found in index_helper: {self.index_helper}"
                    )
        return content_dict

    def update_index_helper(self, content_dict, md_content=None):
        """create a helper for titles with their levels, including all sub-paths"""
        titles_with_levels = content_dict.get("titles_with_levels")
        result = {}
        path_stack = []
        index_helper_iter = iter(self.index_helper)
        for level_info in titles_with_levels:
            level_info["title"] = level_info["title"].strip()
            target_title = level_info["title"]
            if not target_title:
                continue
            level = level_info["level_of_title"]
            target_index = level - 1
            path_stack = path_stack[:target_index]
            path_stack.append(target_title)
            path = tuple(path_stack)
            found_match = False
            while True:
                try:
                    item = next(index_helper_iter)
                    title = list(item.keys())[0]
                    index = list(item.values())[0]
                    if not title:
                        continue
                    if self.match_a_title_and_b_title(title, target_title, str.__eq__):
                        result[path] = index
                        found_match = True
                        break
                except StopIteration:
                    break
            if not found_match:
                raise AssertionError(f"No matching index found for title: {target_title}")
        self.index_helper = result
        self.add_line_number_to_index_helper(md_content=md_content)

    def add_line_number_to_index_helper(self, md_content: str) -> dict:
        """
        Update self.index_helper so each value becomes (page_index, line_number).
        - page_index is the original value stored in index_helper
        - line_number is 1-based, counted in md_content
        """

        # Build a quick lookup: header-text → first line number (1-based)
        header_lines = {}
        for ln, raw in enumerate(md_content.splitlines(), start=1):
            stripped = raw.lstrip()# ignore leading spaces
            if stripped.startswith("#"):
                header_text = stripped.lstrip("#").strip()
                header_text = header_text.lstrip("*").rstrip("*").strip()  # Remove leading/trailing asterisks
                header_lines[header_text] = ln
        # Walk the existing helper and attach line numbers
        for path, page_idx in list(self.index_helper.items()):
            title = path[-1]
            line_num = header_lines.get(title)
            if line_num is None:
                title = title.replace("'", '"').strip()
                line_num = header_lines.get(title)
            self.index_helper[path] = (page_idx, line_num)
        return self.index_helper

    @abstractmethod
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format. To be implemented by subclasses."""
        raise NotImplementedError("This method should be overridden by subclasses.")
