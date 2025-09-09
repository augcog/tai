"""
Pipeline to KB: Complete pipeline from scraping to Knowledge Base with embeddings.

This pipeline includes all three steps:
1. Web scraping using WebScraper(yaml)
2. File conversion using process_folder from api.py
3. Embedding creation using new_embedding_create.py
"""

import logging
import argparse
from pathlib import Path
from typing import Union, Optional
import yaml
import sys
import os


from scraper.Scraper_master.scrapers.web_scraper import WebScraper
from file_conversion_router.services.directory_service import process_folder
from file_conversion_router.new_embedding_create import embedding_create

def load_scraper_config(config_path: Union[str, Path]) -> dict:
    """Load configuration from scraper YAML file."""
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)
    
    # Validate required fields for KB pipeline
    required_fields = ["root_folder", "course_name", "course_code", "db_path"]
    missing_fields = [field for field in required_fields if field not in config]
    if missing_fields:
        raise ValueError(f"Missing required configuration fields: {missing_fields}")
    
    return config

def create_full_kb_pipeline(config_path: Union[str, Path], 
                           output_dir: Optional[Union[str, Path]] = None,
                           embedding_filter: Optional[str] = None,
                           skip_scraping: bool = False) -> None:
    """
    Create a complete knowledge base from scraper configuration file.
    
    Args:
        config_path: Path to scraper YAML configuration file
        output_dir: Optional output directory. If not provided, uses root_folder + "_output"
        embedding_filter: Optional course filter for embeddings (course_name or course_code)
        skip_scraping: If True, skip the scraping step
    """
    config = load_scraper_config(config_path)
    
    # Extract configuration
    input_dir = config["root_folder"]
    course_name = config["course_name"]
    course_code = config["course_code"]
    db_path = config["db_path"]
    log_dir = config.get("log_folder", None)
    
    # Set output directory
    if output_dir is None:
        output_dir = str(Path(input_dir).parent / f"{Path(input_dir).name}_output")
    
    # Use embedding_filter if provided, otherwise use course_name from config
    embedding_course_filter = embedding_filter or course_name
    
    logging.info(f"Starting full KB pipeline for course: {course_name} ({course_code})")
    logging.info(f"Input directory: {input_dir}")
    logging.info(f"Output directory: {output_dir}")
    logging.info(f"Database path: {db_path}")
    
    # Step 1: Web scraping
    if not skip_scraping:
        logging.info("=== Step 1: Web scraping ===")
        try:
            scraper = WebScraper(str(config_path))
            scraper.run()
            logging.info("✓ Web scraping completed successfully")
        except Exception as e:
            logging.error(f"✗ Web scraping failed: {e}")
            raise
    else:
        logging.info("=== Step 1: Skipping web scraping ===")
    
    # Step 2: Convert files and populate database
    logging.info("=== Step 2: Converting files and populating database ===")
    try:
        process_folder(
            input_dir=input_dir,
            output_dir=output_dir,
            course_name=course_name,
            course_code=course_code,
            log_dir=log_dir,
            db_path=db_path,
        )
        logging.info("✓ File conversion completed successfully")
    except Exception as e:
        logging.error(f"✗ File conversion failed: {e}")
        raise
    
    # Step 3: Create embeddings
    logging.info("=== Step 3: Creating embeddings ===")
    try:
        embedding_create(
            db_path=db_path,
            embedding_name=embedding_course_filter,
        )
        logging.info("✓ Embedding creation completed successfully")
    except Exception as e:
        logging.error(f"✗ Embedding creation failed: {e}")
        raise
    
    logging.info("🎉 Full Knowledge Base pipeline completed successfully!")
    logging.info(f"KB is ready at: {db_path}")

def scrape_only(config_path: Union[str, Path]) -> None:
    """
    Run web scraping only.
    
    Args:
        config_path: Path to scraper YAML configuration file
    """
    config = load_scraper_config(config_path)
    course_name = config["course_name"]
    course_code = config["course_code"]
    
    logging.info(f"Starting scraping only for course: {course_name} ({course_code})")
    
    logging.info("=== Web scraping ===")
    try:
        scraper = WebScraper(str(config_path))
        scraper.run()
        logging.info("✓ Web scraping completed successfully")
    except Exception as e:
        logging.error(f"✗ Web scraping failed: {e}")
        raise
    
    logging.info("✅ Scraping pipeline completed successfully!")

def convert_only(config_path: Union[str, Path], 
                output_dir: Optional[Union[str, Path]] = None) -> None:
    """
    Convert files only without scraping or creating embeddings.
    
    Args:
        config_path: Path to scraper YAML configuration file
        output_dir: Optional output directory. If not provided, uses root_folder + "_output"
    """
    config = load_scraper_config(config_path)
    
    # Extract configuration
    input_dir = config["root_folder"]
    course_name = config["course_name"]
    course_code = config["course_code"]
    db_path = config["db_path"]
    log_dir = config.get("log_folder", None)
    
    # Set output directory
    if output_dir is None:
        output_dir = str(Path(input_dir).parent / f"{Path(input_dir).name}_output")
    
    logging.info(f"Starting conversion only for course: {course_name} ({course_code})")
    logging.info(f"Input directory: {input_dir}")
    logging.info(f"Output directory: {output_dir}")
    
    # Convert files using process_folder
    logging.info("=== Converting files and populating database ===")
    try:
        process_folder(
            input_dir=input_dir,
            output_dir=output_dir,
            course_name=course_name,
            course_code=course_code,
            log_dir=log_dir,
            db_path=db_path,
        )
        logging.info("✓ File conversion completed successfully")
    except Exception as e:
        logging.error(f"✗ File conversion failed: {e}")
        raise
    
    logging.info("✅ Conversion pipeline completed successfully!")

def embed_only(db_path: Union[str, Path], 
               embedding_filter: Optional[str] = None) -> None:
    """
    Create embeddings only for an existing KB.
    
    Args:
        db_path: Path to existing SQLite database
        embedding_filter: Optional course filter for embeddings (course_name or course_code)
    """
    logging.info("=== Creating embeddings only ===")
    logging.info(f"Database path: {db_path}")
    
    try:
        embedding_create(
            db_path=db_path,
            embedding_name=embedding_filter,
        )
        logging.info("✓ Embedding creation completed successfully")
    except Exception as e:
        logging.error(f"✗ Embedding creation failed: {e}")
        raise


if __name__ == "__main__":
    create_full_kb_pipeline("/home/bot/bot/yk/YK_final/tai/rag/scraper/Scraper_master/sample_config.yaml")