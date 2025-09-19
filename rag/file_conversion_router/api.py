"""Public API for the file conversion router module."""

import logging

# Import from utility modules
from file_conversion_router.utils.yaml_utils import load_yaml, save_yaml
from file_conversion_router.utils.course_processor import (
    convert_directory,
    process_courses_from_master_config,
    update_master_config_status,
    get_courses_needing_update,
    mark_course_for_update
)
from file_conversion_router.utils.database_merger import (
    merge_course_databases_into_collective,
    merge_all_course_databases_in_directory
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
    'merge_all_course_databases_in_directory'
]


# Example usage (commented out by default)
if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Process all courses marked for update
    # process_courses_from_master_config()

    # Example: Merge course databases after processing
    # After processing individual courses, you can merge them into a collective database:
    #
    # merge_stats = merge_all_course_databases_in_directory(
    #     course_db_directory="/path/to/course/databases",
    #     collective_db_path="/path/to/collective_metadata.db",
    #     db_pattern="*_metadata.db"  # Match files like CS61A_metadata.db
    # )
    # print(f"Merge completed: {merge_stats}")
    #
    # Or merge specific databases:
    # merge_stats = merge_course_databases_into_collective(
    #     course_db_paths=[
    #         "/home/bot/bot/yk/YK_final/courses_out/CS_294-137/CS_294-137_metadata.db",
    #         "/path/to/CS294_metadata.db"
    #     ],
    #     collective_db_path="/home/bot/bot/yk/YK_final/course_yaml/metadata.db"
    # )
    # print(f"Merge completed: {merge_stats}")