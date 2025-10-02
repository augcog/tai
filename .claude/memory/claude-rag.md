# Claude RAG Documentation

This file provides specialized guidance for working with the RAG Pipeline component of the TAI monorepo.

## Component Overview

The RAG Pipeline is a comprehensive document processing pipeline that converts various file types to Markdown format, creates structured pages with chunking for RAG systems, and enriches content with AI-generated educational metadata. Located in `/rag/`, it handles all document ingestion and processing for the TAI platform through three main stages:

1. **File Conversion**: Convert various file types (PDF, HTML, video, notebooks) to Markdown
2. **Page Creation & Chunking**: Structure content into pages with intelligent chunking for retrieval
3. **AI Enhancement** (Optional): Use OpenAI API to generate educational metadata, key concepts, and assessment questions

## Architecture

### Core Technologies
- **Multi-format Support**: PDF, Markdown, HTML, Python, Jupyter notebooks, reStructuredText, Video
- **Advanced OCR**: MinerU, Nougat OCR for complex PDF processing
- **Embedding Models**: BGE-M3, sentence-transformers, custom models
- **Vector Storage**: SQLite with vector extensions, pickle files
- **Service Integration**: FastAPI services for document conversion
- **AI Enhancement**: OpenAI API integration for educational metadata

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

### Document Processing Pipeline
1. **File Conversion**: Format-specific converters transform documents
2. **Text Extraction**: OCR and parsing for various formats
3. **Chunking**: Multiple strategies (recursive, paragraph, semantic)
4. **Embedding Generation**: BGE-M3 multi-modal embeddings
5. **Vector Storage**: SQLite/pickle storage for retrieval

### Database Architecture

- Individual course databases: `{course_code}_metadata.db`
- Collective database: `metadata.db` (merged from course databases)
- SQLite with tables for files, chunks, and embeddings

## Key File Locations

### Core Conversion Services
- `rag/file_conversion_router/` - Core conversion services
- `rag/file_conversion_router/conversion/` - Format-specific converters
- `rag/file_conversion_router/services/` - External service integrations
- `rag/file_conversion_router/embedding/` - Embedding generation
- `rag/file_conversion_router/utils/` - Utilities and helpers
- `rag/file_conversion_router/api.py` - Main API functions
- `rag/file_conversion_router/classes/new_page.py` - Page creation and chunking

### Configuration Files
- `rag/.env` - RAG pipeline configuration
- `rag/example.env` - Environment template
- `rag/file_conversion_router/configs/courses_master_config.yaml` - Master course configuration
- `rag/file_conversion_router/embedding_optimization/src/configs/default_config.yaml` - Optimization config

### Processing Scripts
- `rag/pipeline_to_kb.py` - Main pipeline orchestrator
- `rag/file_conversion_router/embedding/embedding_create.py` - Embedding generation
- `rag/file_conversion_router/embedding/file_embedding_create.py` - File-specific embedding creation
- `rag/file_conversion_router/database_create.py` - Database creation

### External Services
- `rag/scraper/` - Web scraping tools
- `rag/file_organizer/` - File organization utilities

## Development Commands

**Navigate to `rag/` directory first**

### Installation Options
```bash
make install-basic       # Core dependencies only
make install-cv          # Add computer vision support
make install-ocr         # Add OCR capabilities (large download)
make install-full        # All features (5GB+ download)
```

### Document Processing
```bash
make convert INPUT=docs/ OUTPUT=processed/    # Convert documents
make embed INPUT=processed/ OUTPUT=embeddings/  # Generate embeddings
make process INPUT=docs/ OUTPUT=final/        # Full pipeline
```

### Development
```bash
make dev                 # Start RAG development server
make dev-api             # Start FastAPI development server
make test                # Run tests (excludes slow/GPU tests)
make lint                # Run linting checks
make format              # Format code
```

### Model Management
```bash
make download-models     # Download required models
make setup-models        # Setup and verify models
```

## Supported Document Formats

The router handles the following file types with dedicated converters:

| Format               | Extensions                       | Converter               | Features                                 |
| -------------------- | -------------------------------- | ----------------------- | ---------------------------------------- |
| PDF                  | `.pdf`                          | `pdf_converter.py`      | Full text extraction with OCR support for scanned documents |
| Markdown             | `.md`                           | `md_converter.py`       | Enhanced with tree structure preservation |
| HTML                 | `.html`                         | `html_converter.py`     | Preserves structure and formatting       |
| Jupyter Notebooks    | `.ipynb`                        | `notebook_converter.py` | Code and markdown cell extraction        |
| Python               | `.py`                           | `python_converter.py`   | Code documentation and structure analysis |
| reStructuredText     | `.rst`                          | `rst_converter.py`      | Documentation format conversion          |
| Video                | `.mp4`, `.mkv`, `.webm`, `.mov` | `video_converter.py`    | Transcript extraction using Whisper     |

## Environment Configuration

### Document Processing
```bash
INPUT_DIR=/path/to/documents
OUTPUT_DIR=/path/to/processed
CACHE_DIR=/path/to/cache
```

### ML Models
```bash
EMBEDDING_MODEL=BAAI/bge-m3
DEVICE=cuda  # or cpu, mps
```

### Services
```bash
MINERU_SERVICE_URL=http://localhost:8001
NOUGAT_SERVICE_URL=http://localhost:8002
```

### OCR Settings
```bash
OCR_ENABLED=true
OCR_PROVIDER=paddleocr  # or nougat, mineru
```

### OpenAI Integration (Optional)
```bash
OPENAI_API_KEY=your_openai_api_key_here
```

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

## Embedding Generation

### Available Models
```python
MODELS = {
    "bge-m3": "BAAI/bge-m3",           # Multi-modal (recommended)
    "bge-large": "BAAI/bge-large-en",  # English optimized
    "sentence-t5": "sentence-transformers/sentence-t5-base",
    "openai": "text-embedding-ada-002"  # API-based
}
```

### Chunking Strategies
```python
STRATEGIES = {
    "recursive": "Recursive character splitting",
    "paragraph": "Paragraph-based chunks",
    "sentence": "Sentence-based chunks",
    "semantic": "Semantic similarity splitting"
}
```

### Generate Embeddings

```python
from file_conversion_router.embedding.embedding_create import create_embeddings

embeddings = create_embeddings(
    documents_path="/path/to/processed",
    model="bge-m3",
    chunk_size=512,
    overlap=50,
    output_format="sqlite"  # or "pickle"
)
```

## External Service Integration

### MinerU Service
Advanced PDF processing with layout detection:
```python
from file_conversion_router.services.tai_MinerU_service import MinerUService

service = MinerUService(base_url="http://localhost:8001")
result = service.convert_pdf("/path/to/document.pdf")
```

### Nougat OCR Service
Scientific document OCR:
```python
from file_conversion_router.services.tai_nougat_service import NougatService

service = NougatService(base_url="http://localhost:8002")
result = service.process_pdf("/path/to/scientific_paper.pdf")
```

### Task Manager
Distributed processing management:
```python
from file_conversion_router.services.task_manager import TaskManager

manager = TaskManager()
task_id = manager.submit_conversion_task(
    input_path="/docs",
    output_path="/processed",
    converter="pdf"
)
status = manager.get_task_status(task_id)
```

## Testing Strategy

### Test Structure
```
tests/
├── test_file_conversion_router/    # Core conversion tests
├── test_services/                  # Service integration tests
├── test_embedding_optimization/    # Optimization pipeline tests
└── data/                          # Test data and fixtures
```

### Test Categories & Markers
- `slow`: Time-intensive tests
- `cv`: Computer vision tests
- `ocr`: OCR functionality tests
- `gpu`: GPU-required tests
- `integration`: Service integration tests
- `unit`: Unit tests

### Running Tests
```bash
# All tests (excluding slow ones)
make test

# Specific test categories
poetry run pytest tests/test_file_conversion_router/
poetry run pytest -m "not slow and not ocr and not gpu"

# Performance tests
poetry run pytest -m "slow" --durations=10
```

## Optimization Pipeline

The embedding optimization system allows systematic testing of different configurations:

```bash
# Run optimization pipeline
poetry run python file_conversion_router/embedding_optimization/src/pipeline/optimizer.py

# Custom configuration
poetry run python -c "
from embedding_optimization.src.pipeline.builder import PipelineBuilder
pipeline = PipelineBuilder().with_model('bge-m3').with_chunking('recursive').build()
results = pipeline.run('/path/to/docs')
"
```

## Package Management

RAG uses Poetry with workspace-specific virtual environments:

```bash
# Add dependencies (modifies local pyproject.toml)
make add PKG=torch                   # Add latest version
make add PKG="torch==2.0.1"         # Add specific version
make add-dev PKG=pytest             # Add development dependency

# Install optional feature groups
make install-cv                     # Computer vision support
make install-ocr                    # OCR capabilities
make install-video                  # Video processing
make install-web                    # Web scraping

# Add to specific feature groups
make add-cv PKG="opencv-python==4.8.0"    # Add CV package
make add-ocr PKG="paddleocr==2.7.0"       # Add OCR package
```

### Available Extras
| Extra        | Description          | Key Packages                         |
| ------------ | -------------------- | ------------------------------------ |
| `cv`         | Computer Vision      | opencv, albumentations, scikit-image |
| `ocr`        | OCR Processing       | paddleocr, pix2text, rapidocr        |
| `ml-heavy`   | Performance          | accelerate, optimum, onnx            |
| `scientific` | Scientific Computing | scipy, scikit-learn, numba           |
| `nlp`        | NLP Tools            | nltk, spacy, rapidfuzz               |
| `video`      | Video Processing     | moviepy, ffmpeg-python               |
| `web`        | Web Scraping         | playwright, selenium                 |
| `formats`    | Format Support       | python-pptx, easyocr, pypdf          |

## Development Patterns

### Adding New Converters

To add support for a new file format:
1. Create converter in `conversion/` extending `BaseConverter`
2. Implement `to_markdown()` method
3. Register extension in `services/directory_service.py`
4. Add tests for the new converter

```python
from file_conversion_router.conversion.base_converter import BaseConverter

class NewFormatConverter(BaseConverter):
    def to_markdown(self, input_path: str, output_path: str) -> dict:
        # Implementation here
        pass

    def validate_input(self, file_path: str) -> bool:
        # Validation logic
        pass
```

### Caching System
- Use `utils/conversion_cache.py` for intelligent conversion result caching
- Implement batch processing for efficient multi-document handling
- Leverage GPU acceleration for ML operations
- Caching prevents re-conversion of unchanged files

### Configuration Files

#### Course Configuration (YAML)
```yaml
input_dir: /path/to/course/materials
output_dir: /path/to/processed/output
course_name: "Computer Science 61A"
course_code: "CS61A"
db_path: /path/to/CS61A_metadata.db
log_folder: /path/to/logs
```

#### Master Configuration
Located at `configs/courses_master_config.yaml`, tracks all courses with:
- `needs_update`: Boolean flag for processing
- `enabled`: Course active status
- `last_updated`: Timestamp
- `config_path`: Path to course-specific config

### Performance Monitoring
```bash
# Monitor conversion performance
poetry run python file_conversion_router/utils/time_measure.py --input /docs

# Hardware detection
poetry run python file_conversion_router/utils/hardware_detection.py
```

## Common Issues & Troubleshooting

### OCR Dependencies
```bash
# Install OCR dependencies
make install-ocr

# Test OCR installation
poetry run python -c "import paddleocr; print('OCR ready')"
```

### GPU Memory Issues
```bash
# Check GPU status
poetry run python -c "import torch; print(torch.cuda.get_device_properties(0))"

# Reduce batch size in config
export BATCH_SIZE=1
export MAX_LENGTH=512
```

### Model Download Issues
```bash
# Clear model cache
rm -rf ~/.cache/huggingface/

# Download models manually
poetry run python -c "from transformers import AutoModel; AutoModel.from_pretrained('BAAI/bge-m3')"
```

## Advanced Features

- **Caching Mechanism**: Avoids re-conversion of unchanged files
- **Performance Monitoring**: Logs and monitors conversion time for each file
- **Conversion Fidelity**: 95% similarity threshold for test validations
- **Robust Logging**: Detailed logging of conversion processes
- **Ignore Patterns**: Support for `.conversionignore` file to exclude files/folders
- **Parallel Processing**: Support via `TaskManager` for distributed processing

## Security & Performance

### Security Features
- **Input Validation**: Comprehensive file type and content validation
- **Sandboxed Processing**: Isolated conversion environments
- **Resource Limits**: Memory and CPU usage controls
- **Path Traversal Protection**: Secure file handling

### Performance Optimizations
- **Caching System**: Intelligent conversion result caching
- **Batch Processing**: Efficient multi-document handling
- **GPU Acceleration**: CUDA support for ML operations
- **Streaming**: Memory-efficient large file processing
- **Performance Considerations**: Conversion fidelity varies by device (CPU/GPU/MPS), 95% similarity threshold used for test validation