import mimetypes
import os
import re
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Dict, Union

from fastapi import HTTPException, status
from fastapi.responses import FileResponse

from app.config import settings

# Initialize mimetypes
mimetypes.init()

# Define base directory for file storage
if hasattr(settings, "DATA_DIR") and settings.DATA_DIR is not None:
    # Use absolute path if DATA_DIR is absolute, otherwise make it relative to the app root
    if os.path.isabs(settings.DATA_DIR):
        FILE_BASE_DIR = Path(settings.DATA_DIR)
    else:
        # Get the app root directory (3 levels up from this file)
        app_root = Path(__file__).parent.parent.parent.parent.parent.parent
        FILE_BASE_DIR = app_root / settings.DATA_DIR
else:
    # Fallback to test_files directory if it exists
    app_root = Path(__file__).parent.parent.parent.parent.parent.parent
    test_files_dir = app_root / "app" / "core" / "data" / "test_files"
    
    if test_files_dir.exists():
        FILE_BASE_DIR = test_files_dir
    else:
        # Fallback to data directory in app root
        FILE_BASE_DIR = app_root / "data"

print(f"Using file base directory: {FILE_BASE_DIR}")

# Define standard course directory structure
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

# Category based on file extension mapping
FILE_CATEGORY_MAPPING = {
    # Documents
    "pdf": "Document",
    "doc": "Document",
    "docx": "Document",
    "txt": "Document",
    "md": "Document",
    "tex": "Document",
    
    # Assignments
    "ipynb": "Assignment",
    "py": "Assignment",
    "java": "Assignment",
    "c": "Assignment",
    "cpp": "Assignment",
    "js": "Assignment",
    
    # Videos
    "mp4": "Video",
    "avi": "Video",
    "mov": "Video",
    "mkv": "Video",
    "webm": "Video",
    
    # Default
    "default": "Others"
}

# Folder to subdirectory mapping
FOLDER_MAPPING = {
    "Lab Material": "lab_material",
    "Code Script": "code_script",
    "Exams": "exams",
    "Past Projects": "past_projects"
}

# Course mapping for automatic categorization
COURSE_CODE_PATTERN = re.compile(r'(\w+\s*\d+\w*)', re.IGNORECASE)


# Ensure directories exist
def initialize_directories():
    """Initialize the directory structure for file storage"""
    FILE_BASE_DIR.mkdir(exist_ok=True)


# Initialize directories at module load time
initialize_directories()


class LocalFile:
    """Model for file information"""
    def __init__(
            self,
            file_name: str,
            file_path: str,
            mime_type: str,
            size_bytes: int,
            modified_time: str,
            directory: Optional[str] = None,
            thumbnail_url: Optional[str] = None,
            description: Optional[str] = None,
            author: Optional[str] = None,
            created_time: Optional[str] = None,
            last_accessed_time: Optional[str] = None,
            course_code: Optional[str] = None,
            assignment_number: Optional[int] = None,
            lecture_number: Optional[int] = None,
            category: Optional[str] = None,
            folder: Optional[str] = None
    ):
        self.file_name = file_name
        self.file_path = file_path
        self.mime_type = mime_type
        self.size_bytes = size_bytes
        self.modified_time = modified_time
        self.directory = directory
        self.thumbnail_url = thumbnail_url
        self.description = description
        self.author = author
        self.created_time = created_time
        self.last_accessed_time = last_accessed_time
        self.course_code = course_code
        self.assignment_number = assignment_number
        self.lecture_number = lecture_number
        self.category = category
        self.folder = folder

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            "file_name": self.file_name,
            "file_path": self.file_path,
            "mime_type": self.mime_type,
            "size_bytes": self.size_bytes,
            "modified_time": self.modified_time,
            "directory": self.directory,
            "thumbnail_url": self.thumbnail_url,
            "description": self.description,
            "author": self.author,
            "created_time": self.created_time,
            "last_accessed_time": self.last_accessed_time,
            "course_code": self.course_code,
            "assignment_number": self.assignment_number,
            "lecture_number": self.lecture_number,
            "category": self.category,
            "folder": self.folder
        }


class FileDirectory:
    """Model for directory information"""
    def __init__(
            self,
            name: str,
            path: str,
            parent_path: Optional[str] = None,
            is_category: bool = False,
            icon: Optional[str] = None
    ):
        self.name = name
        self.path = path
        self.parent_path = parent_path
        self.is_category = is_category
        self.icon = icon

    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return {
            "name": self.name,
            "path": self.path,
            "parent_path": self.parent_path,
            "is_category": self.is_category,
            "icon": self.icon
        }


def detect_course_code(file_path: str) -> Optional[str]:
    """
    Detect course code from file path using pattern matching
    For course-centric structure, the first directory is typically the course code
    """
    path_parts = Path(file_path).parts
    if len(path_parts) > 0:
        # First try to use the first directory as the course code
        first_dir = path_parts[0]
        if COURSE_CODE_PATTERN.match(first_dir):
            return first_dir
    
    # If not found in first dir, try regex pattern on the full path
    match = COURSE_CODE_PATTERN.search(file_path)
    if match:
        return match.group(1)
    
    return None


def detect_file_category(file_path: str) -> str:
    """
    Determine file category based on extension
    """
    extension = file_path.lower().split('.')[-1] if '.' in file_path else ''
    return FILE_CATEGORY_MAPPING.get(extension, FILE_CATEGORY_MAPPING["default"])


def detect_folder_type(file_path: str) -> Optional[str]:
    """
    Determine folder type based on path
    For course-centric structure, look for folder types in subdirectories
    """
    path_parts = Path(file_path).parts
    
    # Skip the first part which is the course code
    if len(path_parts) >= 3 and path_parts[1] == "documents":
        # Check if the third part (documents subdirectory) is a known folder type
        for folder_name, subdir in FOLDER_MAPPING.items():
            if path_parts[2] == subdir:
                return folder_name
    
    # Try to determine from parent directory name
    if len(path_parts) >= 3:
        parent_dir = path_parts[2].lower()
        
        if "lab" in parent_dir:
            return "Lab Material"
        elif "code" in parent_dir or "script" in parent_dir:
            return "Code Script"
        elif "exam" in parent_dir:
            return "Exams"
        elif "project" in parent_dir:
            return "Past Projects"
    
    return None


def list_files(directory: Optional[str] = None) -> List[LocalFile]:
    """
    List all files in the specified directory or in all directories if not specified
    
    Args:
        directory: Optional subdirectory to list files from
        
    Returns:
        List of LocalFile objects containing file metadata
    """
    files = []

    # Target directory for scanning
    target_dir = FILE_BASE_DIR
    if directory:
        target_dir = FILE_BASE_DIR / directory
        if not target_dir.exists() or not target_dir.is_dir():
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, 
                detail=f"Directory '{directory}' not found"
            )

    # Scan all directories if none specified
    if directory:
        dirs_to_scan = [target_dir]
    else:
        dirs_to_scan = []
        # First level: course directories
        try:
            for course_dir in FILE_BASE_DIR.iterdir():
                if course_dir.is_dir():
                    dirs_to_scan.append(course_dir)
            
            # If no directories found at the base level, add the base directory itself
            if not dirs_to_scan:
                dirs_to_scan = [FILE_BASE_DIR]
                
        except Exception as e:
            # Handle file access errors
            print(f"Error accessing directory {FILE_BASE_DIR}: {str(e)}")
            dirs_to_scan = []

    # Collect file information from each directory
    for dir_path in dirs_to_scan:
        try:
            # Use Path.rglob to scan recursively (equivalent to **/* glob pattern)
            for file_path in dir_path.rglob("*"):
                if file_path.is_file():
                    # Skip stub files used for demonstration
                    if file_path.name.endswith('.stub'):
                        continue
                        
                    # Get file stats
                    stat = file_path.stat()
                    mime_type, _ = mimetypes.guess_type(file_path)
                    
                    # Create relative path from base directory
                    try:
                        rel_path = str(file_path.relative_to(FILE_BASE_DIR))
                    except ValueError:
                        # If file is not under the base directory, skip it
                        continue
                    
                    # Extract additional metadata
                    course_code = detect_course_code(rel_path)
                    category = detect_file_category(rel_path)
                    folder = detect_folder_type(rel_path)
                    
                    # Determine directory part
                    dir_parts = Path(rel_path).parts
                    parent_dir = str(Path(*dir_parts[:-1])) if len(dir_parts) > 1 else ""
                    
                    # Detect assignment or lecture numbers if present
                    assignment_number = None
                    lecture_number = None
                    filename = file_path.stem.lower()
                    
                    # Look for assignment numbers (e.g., "assignment2" or "hw3")
                    if "assignment" in filename or "hw" in filename:
                        numbers = re.findall(r'\d+', filename)
                        if numbers:
                            assignment_number = int(numbers[0])
                    
                    # Look for lecture numbers (e.g., "lecture1" or "lec2")
                    if "lecture" in filename or "lec" in filename:
                        numbers = re.findall(r'\d+', filename)
                        if numbers:
                            lecture_number = int(numbers[0])
                    
                    # For filenames starting with numbers (e.g., "01_Getting_Started")
                    if re.match(r'^\d+_', filename):
                        numbers = re.findall(r'^\d+', filename)
                        if numbers and not lecture_number:
                            lecture_number = int(numbers[0])

                    files.append(LocalFile(
                        file_name=file_path.name,
                        file_path=rel_path,
                        mime_type=mime_type or "application/octet-stream",
                        size_bytes=stat.st_size,
                        modified_time=datetime.fromtimestamp(stat.st_mtime).isoformat(),
                        directory=parent_dir,
                        created_time=datetime.fromtimestamp(stat.st_ctime).isoformat(),
                        last_accessed_time=datetime.fromtimestamp(stat.st_atime).isoformat(),
                        course_code=course_code,
                        assignment_number=assignment_number,
                        lecture_number=lecture_number,
                        category=category,
                        folder=folder
                    ))
        except Exception as e:
            # Handle file access errors for individual directories
            print(f"Error scanning directory {dir_path}: {str(e)}")
            continue

    return files


def list_directories(directory: Optional[str] = None) -> List[FileDirectory]:
    """
    List all directories in the specified directory or in all directories if not specified
    
    Args:
        directory: Optional subdirectory to list directories from
        
    Returns:
        List of FileDirectory objects containing directory metadata
    """
    directories = []

    # Target directory for scanning
    target_dir = FILE_BASE_DIR
    if directory:
        target_dir = FILE_BASE_DIR / directory
        if not target_dir.exists() or not target_dir.is_dir():
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Directory '{directory}' not found"
            )

    # Scan target directory for subdirectories
    for dir_path in target_dir.iterdir():
        if dir_path.is_dir():
            # Determine if this is a course directory (top level) or a subdirectory
            is_course_dir = dir_path.parent == FILE_BASE_DIR
            
            # Determine parent path
            parent_path = None
            if directory:
                parent_path = directory
            
            # Get relative path
            rel_path = str(dir_path.relative_to(FILE_BASE_DIR))
            
            # Determine name and icon based on path and structure
            name = dir_path.name
            icon = "folder"
            
            # If it's a course directory
            if is_course_dir:
                icon = "folder-course"
            # If it's a category directory (e.g., documents, videos)
            elif dir_path.name in COURSE_DIRECTORY_STRUCTURE:
                name = COURSE_DIRECTORY_STRUCTURE[dir_path.name]["display_name"]
                if "document" in dir_path.name.lower():
                    icon = "folder-file"
                elif "video" in dir_path.name.lower():
                    icon = "folder-video"
                elif "audio" in dir_path.name.lower():
                    icon = "folder-audio"
            # If it's a subdirectory within a category
            else:
                for folder_name, subdir in FOLDER_MAPPING.items():
                    if dir_path.name == subdir:
                        name = folder_name
                        icon = f"folder-{subdir.replace('_', '-')}"
            
            directories.append(FileDirectory(
                name=name,
                path=rel_path,
                parent_path=parent_path,
                is_category=not is_course_dir and dir_path.name in COURSE_DIRECTORY_STRUCTURE,
                icon=icon
            ))

    return directories


def get_file(file_path: str) -> FileResponse:
    """
    Retrieve a file by its path
    
    Args:
        file_path: Path to the file, relative to the base data directory
        
    Returns:
        FileResponse: File content with appropriate Content-Type header
    """
    # Construct absolute path and ensure it's within the base directory
    full_path = FILE_BASE_DIR / file_path

    # Security check - ensure path is within FILE_BASE_DIR
    try:
        full_path.relative_to(FILE_BASE_DIR)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: Path is outside the allowed directory"
        )

    # Check if file exists
    if not full_path.exists() or not full_path.is_file():
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"File not found: {file_path}"
        )

    # Determine mimetype
    mime_type, _ = mimetypes.guess_type(full_path)

    # Return file response
    return FileResponse(
        path=str(full_path),
        media_type=mime_type,
        filename=full_path.name
    )


# Make functions available as a module
class LocalStorageModule:
    def list_files(self, directory: Optional[str] = None) -> List[LocalFile]:
        return list_files(directory)

    def list_directories(self, directory: Optional[str] = None) -> List[FileDirectory]:
        return list_directories(directory)

    def get_file(self, file_path: str) -> FileResponse:
        return get_file(file_path)

    # Make models available
    LocalFile = LocalFile
    FileDirectory = FileDirectory


# Module instance
local_storage = LocalStorageModule() 