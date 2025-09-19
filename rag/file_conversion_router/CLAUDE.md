# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

This is the File Conversion Router component of the TAI (Teaching Assistant Intelligence) RAG pipeline, focused on converting various document formats to Markdown with metadata extraction and embedding generation.

## Development Commands

### Setup and Installation

```bash
# Navigate to the parent RAG directory first
cd ..

# Install dependencies using Poetry with local .venv
make install-basic      # Core dependencies only
make install-cv         # Add computer vision support
make install-ocr        # Add OCR capabilities (large download)
make install-full       # All features (5GB+ download)
```

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
# Convert documents
make convert INPUT=docs/ OUTPUT=processed/

# Generate embeddings
make embed INPUT=processed/ OUTPUT=embeddings/

# Full pipeline
make process INPUT=docs/ OUTPUT=final/
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
   - `.md`: Markdown content
   - `.md.tree.txt`: Embedded tree structure
   - `.md.pkl`: Serialized metadata

### Database Architecture

- Individual course databases: `{course_code}_metadata.db`
- Collective database: `metadata.db` (merged from course databases)
- SQLite with tables for files, chunks, and embeddings

## Key API Functions

From `api.py`:
- `convert_directory()`: Process single course directory
- `process_courses_from_master_config()`: Batch process multiple courses
- `merge_course_databases_into_collective()`: Merge course databases
- `get_courses_needing_update()`: Query courses marked for processing
- `mark_course_for_update()`: Flag course for reprocessing

## Configuration

### Course Configuration (YAML)
```yaml
input_dir: /path/to/course/files
output_dir: /path/to/output
course_name: "Computer Science 61A"
course_code: "CS61A"
db_path: /path/to/course_metadata.db
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

## Performance Considerations

- Conversion fidelity varies by device (CPU/GPU/MPS)
- 95% similarity threshold used for test validation
- Caching prevents re-conversion of unchanged files
- Parallel processing support via `TaskManager`