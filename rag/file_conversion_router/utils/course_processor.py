"""Course processing utilities for managing course updates and configurations."""

import logging
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Union

from .yaml_utils import load_yaml, save_yaml
from file_conversion_router.services.directory_service import process_folder


def convert_directory(input_config: Union[str, Path]) -> None:
    """Convert all supported files in the given directory to Markdown format.

    Current supported file types:
    1. PDF
    2. Markdown (To clarify, this markdown includes additional tree structure of original markdown file)

    Args:
        input_config: Path to the configuration YAML file
    """
    data = load_yaml(str(input_config))
    input_dir = data["input_dir"]
    output_dir = data["output_dir"]
    course_name = data["course_name"]
    course_id = data["course_code"]
    log_dir = data.get("log_folder", None)
    db_path = data.get("db_path", None)

    process_folder(
        input_dir,
        output_dir,
        course_name,
        course_id,
        log_dir=log_dir,
        db_path=db_path,
    )


def process_courses_from_master_config(master_config_path: Optional[str] = None) -> None:
    """Process all courses marked for update in the master configuration file.

    Args:
        master_config_path: Path to the master configuration file.
                          Defaults to courses_master_config.yaml in configs directory.
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent.parent / "configs" / "courses_master_config.yaml"

    if not Path(master_config_path).exists():
        raise FileNotFoundError(f"Master config file not found: {master_config_path}")

    master_config = load_yaml(str(master_config_path))
    courses_to_process = []

    # Find courses that need updating
    for course_name, course_info in master_config.get("courses", {}).items():
        if course_info.get("needs_update", False) and course_info.get("enabled", True):
            config_path = course_info.get("config_path")
            if config_path and Path(config_path).exists():
                courses_to_process.append((course_name, config_path))
                logging.info(f"Course '{course_name}' marked for processing")
            else:
                logging.warning(f"Config file not found for course '{course_name}': {config_path}")

    if not courses_to_process:
        logging.info("No courses marked for update")
        return

    logging.info(f"Processing {len(courses_to_process)} courses...")

    # Process each course
    processed_courses = []
    for course_name, config_path in courses_to_process:
        try:
            logging.info(f"Starting processing for course: {course_name}")
            convert_directory(config_path)
            processed_courses.append(course_name)
            logging.info(f"Successfully processed course: {course_name}")
        except Exception as e:
            logging.error(f"Failed to process course '{course_name}': {str(e)}")
            continue

    # Update master config to mark processed courses as completed
    if processed_courses:
        update_master_config_status(str(master_config_path), processed_courses)
        logging.info(f"Updated master config for processed courses: {processed_courses}")


def update_master_config_status(master_config_path: str, processed_courses: List[str]) -> None:
    """Update the master configuration file to mark courses as processed.

    Args:
        master_config_path: Path to the master configuration file
        processed_courses: List of course names that were successfully processed
    """
    master_config = load_yaml(master_config_path)
    current_time = datetime.now().minute

    for course_name in processed_courses:
        if course_name in master_config.get("courses", {}):
            master_config["courses"][course_name]["needs_update"] = False
            master_config["courses"][course_name]["last_updated"] = current_time

    save_yaml(master_config, master_config_path)


def get_courses_needing_update(master_config_path: Optional[str] = None) -> List[Dict[str, Optional[str]]]:
    """Get list of courses that need updating.

    Args:
        master_config_path: Path to the master configuration file

    Returns:
        List of dictionaries containing course information for courses needing update
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent.parent / "configs" / "courses_master_config.yaml"

    master_config = load_yaml(str(master_config_path))
    courses_needing_update = []

    for course_name, course_info in master_config.get("courses", {}).items():
        if course_info.get("needs_update", False) and course_info.get("enabled", True):
            courses_needing_update.append({
                "name": course_name,
                "config_path": course_info.get("config_path"),
                "last_updated": course_info.get("last_updated")
            })

    return courses_needing_update


def mark_course_for_update(course_name: str, master_config_path: Optional[str] = None) -> bool:
    """Mark a specific course for update.

    Args:
        course_name: Name of the course to mark for update
        master_config_path: Path to the master configuration file

    Returns:
        True if successful, False if course not found
    """
    if master_config_path is None:
        master_config_path = Path(__file__).parent.parent / "configs" / "courses_master_config.yaml"

    master_config = load_yaml(str(master_config_path))

    if course_name in master_config.get("courses", {}):
        master_config["courses"][course_name]["needs_update"] = True
        save_yaml(master_config, str(master_config_path))
        logging.info(f"Marked course '{course_name}' for update")
        return True
    else:
        logging.warning(f"Course '{course_name}' not found in master config")
        return False