## File Conversion Router

The File Conversion Router is a comprehensive document processing pipeline that converts various file types to Markdown format, creates structured pages with chunking for RAG systems, and enriches content with AI-generated educational metadata. This scalable tool is designed for processing course materials and building knowledge bases.

### Core Workflow

The system operates through three main stages:

1. **File Conversion**: Convert various file types (PDF, HTML, video, notebooks) to Markdown
2. **Page Creation & Chunking**: Structure content into pages with intelligent chunking for retrieval
3. **AI Enhancement** (Optional): Use OpenAI API to generate educational metadata, key concepts, and assessment questions

### Installation

Ensure Python version 3.10+ is installed.

1. **Install Dependencies**: Navigate to the parent RAG directory and use Poetry:

   ```bash
   cd ..  # Go to rag directory
   make install              # Install core dependencies
   make install-ocr          # Add OCR support (for scanned PDFs)
   make install-cv           # Add computer vision support
   make install-full         # Install all features (5GB+ download)
   ```

2. **GPU Support**: For GPU acceleration with magic-pdf/MinerU:
   - Refer to the [MinerU documentation](https://github.com/opendatalab/MinerU) for GPU setup
   - All necessary dependencies are included in the unified environment

**Note**: This component uses the unified TAI monorepo environment. All dependencies including transformers, magic-pdf, and OCR libraries are managed in the root `pyproject.toml`.

### Configuration

#### Basic Configuration (YAML)
Create a configuration file for your course:

```yaml
input_dir: /path/to/course/materials
output_dir: /path/to/processed/output
course_name: "Computer Science 61A"
course_code: "CS61A"
db_path: /path/to/CS61A_metadata.db
log_folder: /path/to/logs
```

#### OpenAI Integration (Optional)
For AI-enhanced processing, set up your `.env` file:

```bash
OPENAI_API_KEY=your_openai_api_key_here
```

### Supported File Types

The router handles the following file types with dedicated converters:

- **PDF** (`.pdf`) - Full text extraction with OCR support for scanned documents
- **Markdown** (`.md`) - Enhanced with tree structure preservation
- **HTML** (`.html`) - Preserves structure and formatting
- **Jupyter Notebooks** (`.ipynb`) - Code and markdown cell extraction
- **Python** (`.py`) - Code documentation and structure analysis
- **reStructuredText** (`.rst`) - Documentation format conversion
- **Video** (`.mp4`, `.mkv`, `.webm`, `.mov`) - Transcript extraction using Whisper

### Output Structure

For each processed file, the system generates:

1. **Markdown File** (`file.md`)
   - Clean, structured markdown content
   - Preserved formatting and hierarchy

4. **Database Entries**
   - File metadata in SQLite database
   - Chunk records with embeddings
   - Index mappings for efficient retrieval

### Usage Examples

#### 1. Just Conversion (No Embedding)

Convert files to Markdown without creating embeddings:

```python
from file_conversion_router.api import convert_directory

# Convert only - skip embedding generation
result = convert_directory(
    input_config="/path/to/course_config.yaml",
    auto_embed=False  # Don't create embeddings
)
print(f"Converted {result['files_processed']} files")
```

#### 2. Conversion + Automatic Embedding

Convert files and automatically create embeddings:

```python
from file_conversion_router.api import convert_directory

# Convert and embed in one step (recommended)
result = convert_directory(
    input_config="/path/to/course_config.yaml",
    auto_embed=True  # Automatically create both chunk and file embeddings
)
print(f"Processed {result['files_processed']} files")
print(f"Created {result['embeddings_created']} embeddings")
print(f"Embedded {result['chunks_embedded']} chunks")
```

#### 3. Just Embedding (For Already Converted Files)

Create embeddings for courses that have already been converted:

```python
from file_conversion_router.api import create_embeddings_for_course

# Create embeddings for existing course
stats = create_embeddings_for_course(
    db_path="/path/to/CS61A_metadata.db",
    course_code="CS61A",
    data_dir="/path/to/converted/markdown/files",  # Optional: auto-inferred if None
    force_recompute=False  # Set True to regenerate existing embeddings
)
print(f"Files embedded: {stats['files_embedded']}")
print(f"Chunks embedded: {stats['chunks_embedded']}")
```

#### 4. Batch Processing Multiple Courses

Process all courses marked for update in master config:

```python
from file_conversion_router.api import process_courses_from_master_config

# Process all courses with auto-embedding
results = process_courses_from_master_config(
    master_config_path=None,  # Defaults to configs/courses_master_config.yaml
    auto_embed=True  # Set False to skip embedding
)
print(f"Processed: {results['courses_processed']}")
print(f"Failed: {results['courses_failed']}")
```

#### 5. Database Merging

Merge individual course databases into a collective database:

```python
from file_conversion_router.api import merge_course_databases_with_stats

# Merge all course databases with validation
results = merge_course_databases_with_stats(
    master_config_path=None,  # Defaults to configs/courses_master_config.yaml
    exclude_test=True,  # Exclude test/demo databases
    check_embeddings=True  # Validate embeddings before merge
)
print(f"Merged {len(results['included_courses'])} courses")
print(f"Files with embeddings: {results['embedding_stats']['overall']['files_embedded']}")
```

Alternative - merge specific databases:

```python
from file_conversion_router.api import merge_course_databases_into_collective

# Merge specific course databases
merge_course_databases_into_collective(
    course_db_paths=[
        "/path/to/CS61A_metadata.db",
        "/path/to/CS61B_metadata.db"
    ],
    collective_db_path="/path/to/collective_metadata.db"
)
```

#### 6. Check Processing Status

Check embedding and processing status for a course:

```python
from file_conversion_router.api import get_processing_status

# Get detailed status
status = get_processing_status("/path/to/CS61A_metadata.db")
print(f"Files: {status['files_embedded']}/{status['total_files']}")
print(f"Chunks: {status['chunks_embedded']}/{status['total_chunks']}")
print(f"Completion: {status['completion_percentage']:.1f}%")
```

#### 7. Database Validation

Validate database integrity and find issues:

```python
from file_conversion_router.api import validate_database_integrity

# Comprehensive validation
validation = validate_database_integrity(
    db_path="/path/to/collective_metadata.db",
    verbose=True,  # Print detailed report
    save_report="validation_report.txt"  # Optional: save to file
)

if validation["has_critical_issues"]:
    print("Critical issues found!")
    for col in validation["null_columns"]:
        print(f"  - {col['table']}.{col['column']} is completely NULL")
```

#### 8. Course Management

Manage courses in master configuration:

```python
from file_conversion_router.api import (
    mark_course_for_update,
    get_courses_needing_update
)

# Mark a course for re-processing
mark_course_for_update("CS61A")

# Get list of courses needing update
courses = get_courses_needing_update()
print(f"Courses to process: {[c['name'] for c in courses]}")
```

### Complete Processing Pipeline

Here's a typical workflow for processing multiple courses:

```python
from file_conversion_router.api import (
    process_courses_from_master_config,
    merge_course_databases_with_stats,
    validate_database_integrity
)

# Step 1: Convert and embed all courses
print("Step 1: Converting and embedding courses...")
results = process_courses_from_master_config(auto_embed=True)
print(f"Processed: {results['courses_processed']}")

# Step 2: Merge into collective database
print("\nStep 2: Merging course databases...")
merge_results = merge_course_databases_with_stats(
    exclude_test=True,
    check_embeddings=True
)
print(f"Merged {len(merge_results['included_courses'])} courses")

# Step 3: Validate collective database
print("\nStep 3: Validating collective database...")
validation = validate_database_integrity(
    db_path="path/to/collective_metadata.db",
    verbose=True
)
print(f"Validation complete. Critical issues: {validation['has_critical_issues']}")
```

### API Reference

#### Core Workflow Functions

| Function | Description | Key Parameters |
|----------|-------------|----------------|
| `convert_directory()` | Convert files in a directory to Markdown | `input_config`, `auto_embed` |
| `process_courses_from_master_config()` | Batch process multiple courses | `master_config_path`, `auto_embed` |
| `merge_course_databases_with_stats()` | Merge course databases with validation | `master_config_path`, `exclude_test`, `check_embeddings` |

#### Embedding Functions

| Function | Description | Key Parameters |
|----------|-------------|----------------|
| `create_embeddings_for_course()` | Create embeddings for a specific course | `db_path`, `course_code`, `data_dir`, `force_recompute` |
| `embedding_create()` | Create chunk embeddings (chunks.vector) | `db_path`, `course_code` |
| `embed_files_from_markdown()` | Create file embeddings (file.vector) | `db_path`, `data_dir`, `course_filter` |
| `check_embedding_status()` | Check embedding completeness | `db_path`, `course_code` |

#### Database Functions

| Function | Description | Key Parameters |
|----------|-------------|----------------|
| `merge_course_databases_into_collective()` | Merge specific databases | `course_db_paths`, `collective_db_path` |
| `merge_all_course_databases_in_directory()` | Merge databases from directory | `course_db_directory`, `collective_db_path`, `db_pattern` |
| `validate_database_integrity()` | Validate database and find issues | `db_path`, `verbose`, `save_report` |

#### Course Management Functions

| Function | Description | Key Parameters |
|----------|-------------|----------------|
| `mark_course_for_update()` | Mark course for re-processing | `course_name` |
| `get_courses_needing_update()` | Get courses marked for update | `master_config_path` |
| `get_processing_status()` | Get detailed processing status | `db_path` |

### Advanced Features

- **Caching Mechanism**: Avoids re-conversion of unchanged files
- **Performance Monitoring**: Logs and monitors conversion time for each file
- **Conversion Fidelity**: 95% similarity threshold for test validations
- **Robust Logging**: Detailed logging of conversion processes
- **Ignore Patterns**: Support for `.conversionignore` file to exclude files/folders
- **Parallel Processing**: Support via `TaskManager` for distributed processing
- **Automatic Data Directory Inference**: Embeddings can auto-detect output directories from configs

### Testing

Run tests from the parent RAG directory:

```bash
cd ..
make test               # Run all tests
make test-cv           # Test computer vision components
make test-ocr          # Test OCR functionality
```

For more detailed examples and expected outputs, refer to the test suite located at `tests/test_file_conversion_router/test_api.py`.