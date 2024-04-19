# File Conversion Router

## Folder Structure
```text
file_conversion_router/
├── __init__.py
├── api.py  # Simple API interface for external use
├── conversion/
│   ├── __init__.py
│   ├── base_converter.py  # Base class for all converters
│   ├── pdf_to_md.py
│   ├── video_to_md.py
│   ├── text_to_md.py
│   ├── audio_to_md.py
│   ├── latex_to_md.py
├── services/
│   ├── __init__.py
│   ├── task_manager.py  # Handles asynchronous task execution
│   ├── directory_service.py  # Manages directory operations
├── utils/
│   ├── __init__.py
│   ├── file_utils.py  # General utilities for file operations
└── tests/
    ├── __init__.py
    ├── test_api.py
    ├── test_converters.py
    ├── test_task_manager.py
    ├── test_directory_service.py
```
