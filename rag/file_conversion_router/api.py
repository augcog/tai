"""Public API for the file conversion router module."""

import logging

# Import from utility modules
from file_conversion_router.utils.yaml_utils import load_yaml, save_yaml
from file_conversion_router.utils.course_processor import (
    convert_directory,
    process_courses_from_master_config,
    update_master_config_status,
    get_courses_needing_update,
    mark_course_for_update,
    merge_course_databases_from_master_config
)
from file_conversion_router.utils.database_merger import (
    merge_course_databases_into_collective,
    merge_all_course_databases_in_directory,
    merge_databases_by_list
)
from file_conversion_router.embedding.embedding_create import embedding_create
from file_conversion_router.embedding.file_embedding_create import (
    embed_files_from_markdown,
    check_embedding_status
)

# Re-export main functions for backward compatibility
__all__ = [
    'load_yaml',
    'save_yaml',
    'convert_directory',
    'process_courses_from_master_config',
    'update_master_config_status',
    'get_courses_needing_update',
    'mark_course_for_update',
    'merge_course_databases_into_collective',
    'merge_all_course_databases_in_directory',
    'merge_databases_by_list',
    'merge_course_databases_from_master_config',
    'embedding_create',
    'embed_files_from_markdown',
    'check_embedding_status'
]


# Example usage (commented out by default)
if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Process all courses marked for update with auto-embedding enabled
    process_courses_from_master_config(auto_embed=True)

    # Merge all course databases from master config
    # merge_stats = merge_course_databases_from_master_config()

    # Alternative: merge specific databases by providing a list of paths
    # merge_stats = merge_databases_by_list(
    #     course_db_paths=[
    #         "/path/to/CS61A_metadata.db",
    #         "/path/to/CS61B_metadata.db",
    #         "/path/to/CS70_metadata.db"
    #     ],
    #     collective_db_path="/path/to/collective_metadata.db"
    # )
