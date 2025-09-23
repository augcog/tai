# Claude RAG Documentation

This file provides specialized guidance for working with the RAG Pipeline component of the TAI monorepo.

## Component Overview

The RAG Pipeline is an advanced document processing, embedding generation, and file conversion system with multi-modal support. Located in `/rag/`, it handles all document ingestion and processing for the TAI platform.

## Architecture

### Core Technologies
- **Multi-format Support**: PDF, Markdown, HTML, Python, Jupyter notebooks
- **Advanced OCR**: MinerU, Nougat OCR for complex PDF processing
- **Embedding Models**: BGE-M3, sentence-transformers, custom models
- **Vector Storage**: SQLite with vector extensions, pickle files
- **Service Integration**: FastAPI services for document conversion

### Document Processing Pipeline
1. **File Conversion**: Format-specific converters transform documents
2. **Text Extraction**: OCR and parsing for various formats
3. **Chunking**: Multiple strategies (recursive, paragraph, semantic)
4. **Embedding Generation**: BGE-M3 multi-modal embeddings
5. **Vector Storage**: SQLite/pickle storage for retrieval

## Key File Locations

### Core Conversion Services
- `rag/file_conversion_router/` - Core conversion services
- `rag/file_conversion_router/conversion/` - Format-specific converters
- `rag/file_conversion_router/services/` - External service integrations
- `rag/file_conversion_router/embedding/` - Embedding generation
- `rag/file_conversion_router/utils/` - Utilities and helpers

### Configuration Files
- `rag/.env` - RAG pipeline configuration
- `rag/example.env` - Environment template
- `rag/file_conversion_router/embedding_optimization/src/configs/default_config.yaml` - Optimization config

### Processing Scripts
- `rag/pipeline_to_kb.py` - Main pipeline orchestrator
- `rag/file_conversion_router/embedding_create.py` - Embedding generation
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

| Format   | Converter               | Features                                 |
| -------- | ----------------------- | ---------------------------------------- |
| PDF      | `pdf_converter.py`      | OCR, layout detection, table extraction  |
| Markdown | `md_converter.py`       | Syntax parsing, metadata extraction      |
| HTML     | `html_converter.py`     | Clean text extraction, link preservation |
| Python   | `python_converter.py`   | Code parsing, docstring extraction       |
| Jupyter  | `notebook_converter.py` | Cell processing, output handling         |
| Video    | `video_converter.py`    | Transcript extraction, scene detection   |

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

## Conversion Pipeline Usage

### Basic Conversion
```python
from file_conversion_router.api import convert_documents

# Basic conversion
results = convert_documents(
    input_path="/path/to/documents",
    output_path="/path/to/output",
    formats=["pdf", "md"]
)
```

### Advanced Conversion with Options
```python
results = convert_documents(
    input_path="/path/to/documents",
    output_path="/path/to/output",
    formats=["pdf"],
    ocr_enabled=True,
    chunk_size=512,
    overlap=50
)
```

### CLI Usage
```bash
# Convert single file
poetry run rag-convert --input document.pdf --output processed/

# Batch conversion
poetry run rag-convert --input /docs --output /processed --recursive

# With specific options
poetry run rag-convert --input document.pdf --ocr --chunk-size 512
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
```python
from file_conversion_router.conversion.base_converter import BaseConverter

class NewFormatConverter(BaseConverter):
    def convert(self, input_path: str, output_path: str) -> dict:
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