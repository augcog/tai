#!/usr/bin/env python3
"""
Initialize File Storage Directory Structure

This script creates the recommended directory structure for the Local File Retrieval API.
Run this script to set up the directories before adding files.
"""

import os
import sys
from pathlib import Path
import shutil
import argparse

# Define standard directories structure for each course
COURSE_DIRECTORY_STRUCTURE = {
    "documents": {
        "display_name": "Documents",
        "subdirs": [
            "lab_material",
            "code_script",
            "exams",
            "past_projects"
        ]
    },
    "videos": {
        "display_name": "Videos",
        "subdirs": []
    },
    "audios": {
        "display_name": "Audio",
        "subdirs": []
    },
    "others": {
        "display_name": "Others",
        "subdirs": []
    }
}

# Define courses
EXAMPLE_COURSES = [
    "CS61A",
    "CS61B",
    "CS170"
]


def initialize_directories(base_dir: Path, courses=None, clean=False):
    """
    Initialize the directory structure for file storage
    
    Args:
        base_dir: Base directory for file storage
        courses: List of course codes to create subdirectories for
        clean: If True, clean the base directory before initializing
    """
    # Convert to Path object if string
    base_dir = Path(base_dir)
    
    # Clean if requested
    if clean and base_dir.exists():
        print(f"Cleaning directory: {base_dir}")
        shutil.rmtree(base_dir)
    
    # Create base directory
    print(f"Creating base directory: {base_dir}")
    base_dir.mkdir(exist_ok=True)
    
    # Default courses if none specified
    if not courses:
        courses = EXAMPLE_COURSES
    
    # Create course directories and their internal structure
    for course in courses:
        course_dir = base_dir / course
        print(f"Creating course directory: {course_dir}")
        course_dir.mkdir(exist_ok=True)
        
        # Create main directories and subdirectories for each course
        for main_dir, config in COURSE_DIRECTORY_STRUCTURE.items():
            main_dir_path = course_dir / main_dir
            print(f"Creating directory: {main_dir_path}")
            main_dir_path.mkdir(exist_ok=True)
            
            # Create subdirectories
            for subdir in config["subdirs"]:
                subdir_path = main_dir_path / subdir
                print(f"Creating subdirectory: {subdir_path}")
                subdir_path.mkdir(exist_ok=True)


def create_example_files(base_dir: Path):
    """
    Create example placeholder files for demonstration
    """
    base_dir = Path(base_dir)
    
    # Create example text file in CS61A/documents/lab_material
    lab_dir = base_dir / "CS61A" / "documents" / "lab_material"
    example_file = lab_dir / "01_Getting_Started_Guide.txt"
    with open(example_file, "w") as f:
        f.write("This is an example document for CS61A course.\n")
        f.write("Replace this with actual content.\n")
    print(f"Created example file: {example_file}")
    
    # Create example code file in CS61A/documents/code_script
    code_dir = base_dir / "CS61A" / "documents" / "code_script"
    example_code = code_dir / "example_code.py"
    with open(example_code, "w") as f:
        f.write("# Example Python code for CS61A\n\n")
        f.write("def factorial(n):\n")
        f.write("    if n <= 1:\n")
        f.write("        return 1\n")
        f.write("    return n * factorial(n-1)\n\n")
        f.write("# Replace this with actual course code\n")
    print(f"Created example code: {example_code}")
    
    # Create example video file stub in CS61A/videos
    video_dir = base_dir / "CS61A" / "videos"
    example_video_stub = video_dir / "02_Lab_Instructions.mp4.stub"
    with open(example_video_stub, "w") as f:
        f.write("This is a stub for a video file. Replace with actual MP4 file.\n")
        f.write("The .stub extension is just for demonstration and should be removed for real files.\n")
    print(f"Created example video stub: {example_video_stub}")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Initialize file storage directory structure for the Local File Retrieval API"
    )
    parser.add_argument(
        "--base-dir", 
        type=str, 
        default="data",
        help="Base directory for file storage (default: data)"
    )
    parser.add_argument(
        "--courses", 
        nargs="+",
        default=EXAMPLE_COURSES,
        help="List of course codes to create subdirectories for"
    )
    parser.add_argument(
        "--clean", 
        action="store_true",
        help="Clean the base directory before initializing"
    )
    parser.add_argument(
        "--examples", 
        action="store_true",
        help="Create example placeholder files"
    )
    
    args = parser.parse_args()
    
    # Initialize directories
    initialize_directories(args.base_dir, args.courses, args.clean)
    
    # Create example files if requested
    if args.examples:
        create_example_files(args.base_dir)
    
    print("\nDirectory structure initialized successfully!")
    print("You can now add your files to the appropriate directories.")
    print("See docs/local_file_api.md for more information.")
    print("\nExample course structure:")
    print(f"data/CS61A/documents/lab_material/01_Getting_Started_Guide.txt")
    print(f"data/CS61A/documents/code_script/example_code.py")
    print(f"data/CS61A/videos/02_Lab_Instructions.mp4")


if __name__ == "__main__":
    main() 