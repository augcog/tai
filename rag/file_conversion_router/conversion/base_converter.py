"""Base classes for all file type converters."""
import string
import logging
from abc import ABC, abstractmethod
# from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from shutil import copy2
# from threading import Lock
from typing import Dict, List, Union

import yaml
from file_conversion_router.classes.chunk import Chunk
from file_conversion_router.classes.new_page import Page
from file_conversion_router.classes.vidpage import VidPage
from file_conversion_router.embedding_optimization.src.pipeline.optimizer import (
    EmbeddingOptimizer,
)
from file_conversion_router.utils.logger import (
    conversion_logger,
    logger,
    content_logger,
)
from file_conversion_router.utils.utils import (
    calculate_hash,
    ensure_path,
    check_url,
)
from file_conversion_router.utils.conversion_cache import ConversionCache
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
        self, course_name, course_id, optimizer_config_path: Union[str, Path] = None
    ):
        self.index_helper = None
        self.course_name = course_name
        self.course_id = course_id
        self._md_parser = None

        self._md_path = None
        self._pkl_path = None

        self._logger = logger
        self._content_logger = content_logger

        self.cache = ConversionCache

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
        self, input_path: Union[str, Path], output_folder: Union[str, Path], input_root: Union[str, Path] = None
    ) -> None:
        """Convert an input file to 3 files: Markdown, tree txt, and pkl file, under the output folder.

        Args:
            input_path: The path for a single file to be converted. e.g. 'path/to/file.txt'
            output_folder: The folder where the output files will be saved. e.g. 'path/to/output_folder'
                other files will be saved in the output folder, e.g.:
                - 'path/to/output_folder/file.md'
                - 'path/to/output_folder/file.md.tree.txt'
                - 'path/to/output_folder/file.md.pkl'
            input_root: The root folder of the input file, used to calculate the relative path of the input file.
        """
        self.file_name=input_path.name
        self.relative_path = input_path.relative_to(input_root)
        input_path, output_folder = ensure_path(input_path), ensure_path(output_folder)
        if not input_path.exists():
            self._logger.error(f"The file {input_path} does not exist.")
            raise FileNotFoundError(f"The file {input_path} does not exist.")

        self._setup_output_paths(input_path, output_folder)

        file_hash = calculate_hash(input_path)
        cached_paths = self.cache.get_cached_paths(file_hash)
        same_version = (
            self.cache.version == ConversionCache.get_file_conversion_version(file_hash)
        )
        if (
            cached_paths
            and all(Path(path).exists() for path in cached_paths)
            and same_version
        ):
            self._logger.info(
                f"Cached result found, using cached files for input path: {input_path} "
                f"in output folder: {output_folder}."
                f"\n Cached content are: {[str(path) for path in cached_paths]}."
            )
            self._content_logger.warning(
                f"Cached result found, using cached files for input path: {input_path} "
            )
            self._use_cached_files(cached_paths, output_folder)
            return
        # TODO: this is one file conversion, no need to use future
        self._convert_and_cache(input_path, output_folder, file_hash)
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
        cached_paths = self.cache.get_cached_paths(file_hash)
        self._logger.info(
            f"Future completed, using cached files for input path: {input_path} "
            f"in output folder: {output_folder}."
            f"\n Cached content are: {[str(path) for path in cached_paths]}."
        )
        self._use_cached_files(cached_paths, output_folder)

    @conversion_logger
    def _convert_to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format."""
        self._to_markdown(input_path, output_path)

    def _convert_to_page(self, input_path: Path, output_path: Path) -> Page:
        page = self._to_page(input_path, output_path)
        return page

    def _setup_output_paths(
        self, input_path: Union[str, Path], output_folder: Union[str, Path]
    ) -> None:
        """Set up the output paths for the Markdown, tree txt, and pkl files."""
        input_path = ensure_path(input_path)
        output_folder = ensure_path(output_folder)
        self.file_name = input_path.name
        # self.relative_path = input_path.relative_to(output_folder)
        self._md_path = ensure_path(output_folder / f"{input_path.stem}.md")
        # TODO: current MarkdownParser does not support custom output paths,
        #  below paths are only used for caching purposes at the moment,
        #  since the markdown parser generates below file paths by default
        self._pkl_path = ensure_path(output_folder / f"{input_path.stem}.pkl")

    def _convert_and_cache(
        self, input_path: Path, output_folder: Path, file_hash: str
    ) -> List[Path]:
        self._setup_output_paths(input_path, output_folder)
        # This method embeds the abstract method `_to_markdown`, which needs to be implemented by the child classes.
        _, conversion_time = self._perform_conversion(input_path, output_folder)
        paths = [self._md_path, self._pkl_path]
        assert all(path.exists() for path in paths), (
            "Not all output files were generated."
        )
        self.cache.set_cache_and_time(
            file_hash, str(input_path), paths, conversion_time
        )
        logger.info(f"cached into {self.cache._cache_file_path}")
        return paths

    def _use_cached_files(self, cached_paths: List[Path], output_folder: Path) -> None:
        """Use cached files and copy them to the specified output folder, avoiding self-copying."""
        output_folder = ensure_path(output_folder)
        output_folder.mkdir(parents=True, exist_ok=True)

        md_path, pkl_path = cached_paths
        correct_file_name = self._md_path.stem

        for path, suffix in zip((md_path, pkl_path), (".md", ".pkl")):
            des_path = output_folder / f"{correct_file_name}{suffix}"
            path = Path(path)
            # Prevent self-copying if source and destination are identical
            if path.resolve() == des_path.resolve():
                self._logger.info(
                    f"Skipping self-copy: {path} is already in {output_folder}."
                )
                continue

            des_path = Path(copy2(path, output_folder))
            des_path.rename(output_folder / f"{correct_file_name}{suffix}")
            self._logger.info(f"Copied cached file from {path} to {des_path}.")

    def _read_metadata(self, metadata_path: Path) -> dict:
        """Read metadata from file or return mocked data if file doesn't exist."""
        if metadata_path.exists():
            try:
                with open(metadata_path, "r") as metadata_file:
                    return yaml.safe_load(metadata_file)
            except Exception as e:
                self._logger.error(f"Error reading metadata file: {str(e)}")
                return self._get_mocked_metadata()
        else:
            self._logger.warning(
                f"Metadata file not found: {metadata_path}. Using mocked metadata."
            )
            return self._get_mocked_metadata()

    @staticmethod
    def _get_mocked_metadata() -> dict:
        """Return mocked metadata when the actual metadata file is missing."""
        return {
            "URL": "",
        }

    @conversion_logger
    def _perform_conversion(self, input_path: Path, output_folder: Path) -> None:
        """Perform the file conversion process."""
        logging.getLogger().setLevel(logging.INFO)

        logger.info(f"🚀 Starting conversion for {input_path}")
        if not output_folder.exists():
            output_folder.mkdir(parents=True, exist_ok=True)
            logger.warning(
                f"Output folder did not exist, it's now created: {output_folder}"
            )
        logger.info(f"📄 Expected Markdown Path: {self._md_path}")
        logger.info(f"🛠️ Expected Pickle Path: {self._pkl_path}")
        # try:
        page = self._to_page(input_path, self._md_path)
        logger.info("✅ Page conversion successful.")
        # TODO: when chunks are created, instead of save them to pkl, create data base in base_converter.py and save them to database. Consider it is in thead to avoid blocking other addding tasks.
        page.to_chunk()
        logger.info("✅ Successfully converted page content to chunks.")

        page.chunks_to_pkl(str(self._pkl_path))

    def _optimize_markdown_content(self, page: Page, original_content: str) -> None:
        """Optimize the Markdown content and combine enhanced and original versions."""
        result = self.optimizer.process_markdown(original_content)
        if result.success:
            enhanced_content = result.content
            # Combine enhanced and original content with clear headers
            """Uncomment below line after have way to deactivate optimizer"""
            combined_content = (
                # "# TAI Embedding Optimized Content\n\n"
                # f"{enhanced_content}\n\n"
                # "# Original Content\n\n"
                f"{original_content}"
            )
            # Update the page content with combined content
            page.content["text"] = combined_content

            # Resave the combined Markdown content
            with open(self._md_path, "w", encoding="utf-8") as md_file:
                md_file.write(combined_content)
            self._logger.info(
                f"Enhanced and original Markdown saved to {self._md_path}"
            )
        else:
            self._logger.error(f"Failed to optimize Markdown: {result.error}")

    def _optimize_chunks(self, original_chunks: List[Chunk]) -> List[Chunk]:
        """Optimize each chunk and combine enhanced and original versions."""
        optimized_chunks = self.optimizer.process_chunks(original_chunks)
        combined_chunks = []

        for original_chunk, optimized_chunk in zip(original_chunks, optimized_chunks):
            # Combine enhanced and original chunk content with clear headers
            """Uncomment below line after have way to deactivate optimizer"""
            combined_chunk_content = (
                # "## TAI Embedding Optimized Chunk\n\n"
                # f"{optimized_chunk.content}\n\n"
                # "## Original Chunk\n\n"
                f"{original_chunk.content}"
            )

            # Create a new Chunk instance with combined content
            combined_chunk = Chunk(
                content=combined_chunk_content,
                titles=original_chunk.titles,
                chunk_url=original_chunk.chunk_url,
            )
            combined_chunks.append(combined_chunk)
        return combined_chunks

    def _check_page_content(self, page: Page, input_path: Path) -> bool:
        content_length_threshold = Page.PAGE_LENGTH_THRESHOLD
        content_length = len(page.content.get("text", ""))
        page_url = page.page_url
        url_state = check_url(page_url)
        if content_length < content_length_threshold:
            self._content_logger.warning(
                f"File: {input_path} removed. Page has content length: {content_length} "
                f"less than threshold: {content_length_threshold}"
            )
            return False

        if url_state != 200:
            self._content_logger.error(f"File: {input_path} has url state: {url_state}")
        else:
            self._content_logger.info(
                f"File: {input_path} has content length {content_length}, "
                f"url state: {url_state}"
            )
        return True

    def _to_page(self, input_path: Path, output_path: Path) -> Page:
        # Ensure the output directory exists
        output_path.parent.mkdir(parents=True, exist_ok=True)
        stem = input_path.stem
        self.file_type = input_path.suffix.lstrip(".")
        md_path = self._to_markdown(input_path, output_path)
        metadata_path = input_path.with_name(f"{input_path.stem}_metadata.yaml")
        if md_path:
            structured_md, content_dict = self.apply_markdown_structure(input_md_path=md_path, file_type=self.file_type)
            with open(md_path, "w", encoding="utf-8") as md_file:
                md_file.write(structured_md)
        else:
            structured_md = ""
            content_dict = {}
        metadata_content = self._read_metadata(metadata_path)
        metadata_content={'URL': metadata_content.get('URL', '')}
        metadata = self._put_content_dict_to_metadata(
            content_dict=content_dict,
            metadata_content=metadata_content,
        )
        with open(metadata_path, "w", encoding="utf-8") as yaml_file:
            yaml.safe_dump(metadata, yaml_file, allow_unicode=True)
        url = metadata_content.get("URL")
        content = {"text": structured_md}
        return Page(
            filetype=self.file_type,
            content=content,
            page_name=stem,
            page_url=url,
            index_helper=self.index_helper,
        )

    def _put_content_dict_to_metadata(self, content_dict: dict, metadata_content: dict) -> dict:
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
        b_titles = b_titles.translate(str.maketrans('', '', string.punctuation)).lower().strip()
        return operator(a_titles, b_titles)

    def process_problems(self, content_dict):
        # Return just the list of problems, not a dictionary
        problems_list = []
        for problem in content_dict['problems']:
            processed_problem = {}
            for title in self.index_helper:
                if self.match_a_title_and_b_title(title,problem['ID'], str.__contains__):
                    processed_problem['problem_index'] = self.index_helper[title]
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
        if header_levels == 0 and file_type == "mp4":
            content_dict = get_structured_content_without_title(
                md_content=content_text,
                file_name=file_name,
                course_name=self.course_name,
            )
            new_md = apply_structure_for_no_title(
                md_content=content_text, content_dict=content_dict
            )
        elif file_type == "ipynb":
            content_dict = get_strutured_content_for_ipynb(
                md_content=content_text,
                file_name=file_name,
                course_name=self.course_name,
            )
            content_dict=self.update_content_dict_titles_with_levels(
                content_dict=content_dict, content_text=content_text
            )
            self.fix_index_helper_with_titles_with_level(content_dict)
            new_md =content_text

        elif header_levels == 1 and file_type == "pdf":
            content_dict = get_structured_content_with_one_title_level(
                md_content=content_text,
                file_name=file_name,
                course_name=self.course_name,
            )
            new_md = apply_structure_for_one_title(
                md_content=content_text, content_dict=content_dict
            )
        else:
            content_dict = get_only_key_concepts(
                md_content=content_text,
                file_name=file_name,
                course_name=self.course_name,
            )
            content_dict=self.update_content_dict_titles_with_levels(
                content_dict=content_dict, content_text=content_text
            )
            new_md=content_text

        content_dict = self.add_source_section_index(content_dict)
        return new_md, content_dict


    def generate_index_helper(self, md):
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

    def add_source_section_index(self, content_dict: dict):
        self.update_index_helper(content_dict)
        if 'key_concepts' in content_dict:
            for concept in content_dict['key_concepts']:
                source_title = concept['source_section_title']
                # source_title = source_title.strip("#").strip()
                # source_title = source_title.strip("*").strip()
                found = False
                for titles in self.index_helper.keys():
                    if self.match_a_title_and_b_title(titles, source_title, str.__contains__):
                        concept['source_section_title'] = titles
                        concept['source_section_index'] = self.index_helper[titles]
                        found = True
                        break
                if not found:
                    raise ValueError(
                        f"Source section title '{source_title}' not found in index_helper: {self.index_helper}"
                    )
        return content_dict

    def update_index_helper(self, content_dict):
        """create a helper for titles with their levels, including all sub-paths"""
        titles_with_levels = content_dict.get("titles_with_levels")
        result = {}
        path_stack = []
        for item, level_info in zip(self.index_helper, titles_with_levels):
            title = list(item.keys())[0]
            index = list(item.values())[0]
            if not title:
                continue
            level_info["title"]=level_info["title"].strip()
            assert(title == level_info["title"]), (
                f"Title mismatch: {title} != {level_info['title']}"
            )
            level = level_info["level_of_title"]
            target_index = level - 1
            path_stack = path_stack[:target_index]
            path_stack.append(title)
            path = " > ".join(path_stack)
            result[path] = index

        self.index_helper = result


    @abstractmethod
    def _to_markdown(self, input_path: Path, output_path: Path) -> None:
        """Convert the input file to Expected Markdown format. To be implemented by subclasses."""
        raise NotImplementedError("This method should be overridden by subclasses.")
