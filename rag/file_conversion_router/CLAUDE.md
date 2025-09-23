# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

The File Conversion Router is a comprehensive document processing pipeline that converts various file types to Markdown format, creates structured pages with chunking for RAG systems, and enriches content with AI-generated educational metadata. This scalable tool is designed for processing course materials and building knowledge bases.

### Core Workflow

The system operates through three main stages:

1. **File Conversion**: Convert various file types (PDF, HTML, video, notebooks) to Markdown
2. **Page Creation & Chunking**: Structure content into pages with intelligent chunking for retrieval
3. **AI Enhancement** (Optional): Use OpenAI API to generate educational metadata, key concepts, and assessment questions

## Development Commands

### Setup and Installation

Ensure Python version 3.10+ is installed.

```bash
# Navigate to the parent RAG directory first
cd ..

# Install dependencies using Poetry with local .venv
make install            # Install core dependencies
make install-ocr        # Add OCR support (for scanned PDFs)
make install-cv         # Add computer vision support
make install-full       # Install all features (5GB+ download)
```

**GPU Support**: For GPU acceleration with magic-pdf/MinerU:
- Refer to the [MinerU documentation](https://github.com/opendatalab/MinerU) for GPU setup
- All necessary dependencies are included in the unified environment

**Note**: This component uses the unified TAI monorepo environment. All dependencies including transformers, magic-pdf, and OCR libraries are managed in the root `pyproject.toml`.

### Development and Testing

```bash
# From the RAG directory
make dev                # Start development server
make test               # Run tests (excludes slow/GPU tests)
make lint               # Run linting checks
make format             # Format code

# Run specific test categories
make test-cv            # Computer vision tests
make test-ocr           # OCR tests
make test-ml            # ML model tests
```

### Document Processing

```bash
# From the RAG directory
make convert INPUT=docs/ OUTPUT=processed/    # Convert documents
make embed INPUT=processed/ OUTPUT=embeddings/  # Generate embeddings
make process INPUT=docs/ OUTPUT=final/        # Full pipeline
```

## Architecture

### Core Components

The system follows a modular converter pattern where each file type has a dedicated converter that extends `BaseConverter`:

- **Conversion Module** (`conversion/`): Contains converters for PDF, Markdown, HTML, Python, Notebooks, RST, and video files. Each converter implements `to_markdown()` method for format-specific conversion logic.

- **Services** (`services/`): Orchestrates the conversion pipeline with `directory_service.py` handling folder processing and database updates.

- **Utils** (`utils/`):
  - `course_processor.py`: Manages course-level batch processing and master configuration updates
  - `database_merger.py`: Merges individual course databases into collective database
  - `yaml_utils.py`: Configuration file handling
  - `conversion_cache.py`: Caching mechanism for avoiding re-conversion

### Processing Flow

1. **Input**: Configuration YAML specifies input/output directories, course details, and database paths
2. **Conversion**: Files are processed through appropriate converters based on extension
3. **Output**: Each file generates:
   - **Markdown File** (`file.md`): Clean, structured markdown content with preserved formatting and hierarchy
   - **Database Entries**: File metadata in SQLite database, chunk records with embeddings, index mappings for efficient retrieval

### Supported File Types

The router handles the following file types with dedicated converters:

- **PDF** (`.pdf`) - Full text extraction with OCR support for scanned documents
- **Markdown** (`.md`) - Enhanced with tree structure preservation
- **HTML** (`.html`) - Preserves structure and formatting
- **Jupyter Notebooks** (`.ipynb`) - Code and markdown cell extraction
- **Python** (`.py`) - Code documentation and structure analysis
- **reStructuredText** (`.rst`) - Documentation format conversion
- **Video** (`.mp4`, `.mkv`, `.webm`, `.mov`) - Transcript extraction using Whisper

### Database Architecture

- Individual course databases: `{course_code}_metadata.db`
- Collective database: `metadata.db` (merged from course databases)
- SQLite with tables for files, chunks, and embeddings

## Key API Functions

From `api.py`:
- `convert_directory()`: Process single course directory
- `process_courses_from_master_config()`: Batch process multiple courses
- `merge_course_databases_into_collective()`: Merge course databases
- `merge_all_course_databases_in_directory()`: Merge course databases from directory
- `get_courses_needing_update()`: Query courses marked for processing
- `mark_course_for_update()`: Flag course for reprocessing

## Usage Examples

### Basic File Conversion

```python
from file_conversion_router.api import convert_directory

# Convert a single course directory
convert_directory("/path/to/course_config.yaml")
```

### Batch Processing Multiple Courses

```python
from file_conversion_router.api import process_courses_from_master_config

# Process all courses marked for update
process_courses_from_master_config("/path/to/master_config.yaml")
```

### Page Creation with Chunking

```python
from file_conversion_router.classes.new_page import Page

# Create page from converted markdown
page = Page(
    course_name="CS61A",
    course_code="CS61A",
    filetype="pdf",
    content={'text': markdown_content},
    index_helper=title_index_dict
)

# Generate chunks for RAG
page.to_chunk()
page.chunks_to_pkl(output_path="/path/to/output.pkl")
```

### Database Merging

```python
from file_conversion_router.api import merge_all_course_databases_in_directory

# Merge individual course databases into collective
merge_stats = merge_all_course_databases_in_directory(
    course_db_directory="/path/to/course/databases",
    collective_db_path="/path/to/collective_metadata.db",
    db_pattern="*_metadata.db"
)
```

### Complete Processing Pipeline

```python
from file_conversion_router.api import convert_directory
from file_conversion_router.utils.database_merger import merge_course_databases_into_collective

# Step 1: Convert documents
convert_directory("/path/to/CS61A_config.yaml")
convert_directory("/path/to/CS61B_config.yaml")

# Step 2: Merge databases
merge_course_databases_into_collective(
    course_db_paths=[
        "/path/to/CS61A_metadata.db",
        "/path/to/CS61B_metadata.db"
    ],
    collective_db_path="/path/to/all_courses_metadata.db"
)
```

## Configuration

### Course Configuration (YAML)
```yaml
input_dir: /path/to/course/materials
output_dir: /path/to/processed/output
course_name: "Computer Science 61A"
course_code: "CS61A"
db_path: /path/to/CS61A_metadata.db
log_folder: /path/to/logs
```

### OpenAI Integration (Optional)
For AI-enhanced processing, set up your `.env` file:

```bash
OPENAI_API_KEY=your_openai_api_key_here
```

### AI-Enhanced Processing Example

```python
from file_conversion_router.utils.title_handle import get_strutured_content_for_ipynb

# Generate educational metadata for notebook content
structured_content = get_strutured_content_for_ipynb(
    md_content=markdown_text,
    file_name="lecture_01.ipynb",
    course_name="Introduction to Python",
    index_helper=section_index
)
# Returns JSON with key concepts, check-in questions, and problems
```

### Master Configuration
Located at `configs/courses_master_config.yaml`, tracks all courses with:
- `needs_update`: Boolean flag for processing
- `enabled`: Course active status
- `last_updated`: Timestamp
- `config_path`: Path to course-specific config

## Extending the System

To add support for a new file format:
1. Create converter in `conversion/` extending `BaseConverter`
2. Implement `to_markdown()` method
3. Register extension in `services/directory_service.py`
4. Add tests for the new converter

## Advanced Features

- **Caching Mechanism**: Avoids re-conversion of unchanged files
- **Performance Monitoring**: Logs and monitors conversion time for each file
- **Conversion Fidelity**: 95% similarity threshold for test validations
- **Robust Logging**: Detailed logging of conversion processes
- **Ignore Patterns**: Support for `.conversionignore` file to exclude files/folders
- **Parallel Processing**: Support via `TaskManager` for distributed processing

## Testing

Run tests from the parent RAG directory:

```bash
cd ..
make test               # Run all tests
make test-cv           # Test computer vision components
make test-ocr          # Test OCR functionality
```

For more detailed examples and expected outputs, refer to the test suite located at `tests/test_file_conversion_router/test_api.py`.

## Performance Considerations

- Conversion fidelity varies by device (CPU/GPU/MPS)
- 95% similarity threshold used for test validation
- Caching prevents re-conversion of unchanged files
- Parallel processing support via `TaskManager`