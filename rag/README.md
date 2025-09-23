# TAI RAG Pipeline

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.3+-red.svg)](https://pytorch.org/)
[![Poetry](https://img.shields.io/badge/poetry-1.4+-blue.svg)](https://python-poetry.org/)

Advanced RAG (Retrieval-Augmented Generation) pipeline for document processing, embedding generation, and file conversion with multi-modal support.

## 🚀 Quick Start

```bash
# Install dependencies
make install

# Start RAG development tools
make dev

# Convert documents to embeddings
make convert

# Run tests
make test

```

## 🏗️ Architecture

The TAI RAG Pipeline provides comprehensive document processing capabilities:

- **Multi-format Support**: PDF, Markdown, HTML, Python, Jupyter notebooks
- **Advanced OCR**: MinerU, Nougat OCR for complex PDF processing
- **Embedding Models**: BGE-M3, sentence-transformers, custom models
- **Vector Storage**: SQLite with vector extensions, pickle files
- **Service Integration**: FastAPI services for document conversion

## 📁 Project Structure

```
rag/
├── file_conversion_router/     # Core conversion services
│   ├── conversion/            # Format-specific converters
│   ├── services/              # External service integrations
│   ├── embedding/             # Embedding generation
│   └── utils/                 # Utilities and helpers
├── scraper/                   # Web scraping tools
├── tests/                     # Test suites
└── src/                       # Source packages
```

## 🎯 Core Features

### 1. **Document Conversion**

- **PDF Processing**: Advanced OCR with MinerU and Nougat
- **Multi-format Support**: Markdown, HTML, Python, Jupyter
- **Metadata Extraction**: Automatic title, author, and structure detection
- **Chunking Strategies**: Recursive, paragraph, sentence-based splitting

### 2. **Embedding Generation**

- **State-of-the-art Models**: BGE-M3, sentence-transformers
- **Multi-modal Embeddings**: Dense, sparse, and ColBERT
- **Optimization Pipeline**: Configurable processing workflows
- **Vector Storage**: Efficient storage and retrieval systems

### 3. **Service Architecture**

- **FastAPI Integration**: RESTful APIs for all operations
- **External Services**: MinerU, Nougat OCR integration
- **Task Management**: Distributed processing capabilities
- **Caching System**: Intelligent conversion caching

## 🛠️ Development Commands

### Installation & Setup

```bash
make install              # Install core dependencies
make install-cv           # Install computer vision extras
make install-ocr          # Install OCR dependencies
make install-full         # Install all optional dependencies
make clean                # Clean build artifacts
```

### Development

```bash
make dev                  # Start development server
make test                 # Run tests (excludes slow/GPU tests)
make lint                 # Run linting
make format               # Format code
```

### Document Processing

```bash
make convert              # Run document conversion pipeline
make embed                # Generate embeddings
make process              # Full processing pipeline
```

## 📦 Package Management

The RAG pipeline uses Poetry with workspace-specific virtual environments. Each workspace maintains its own isolated environment:

```bash
# Add dependencies (modifies local pyproject.toml)
make add PKG=torch                   # Add latest version
make add PKG="torch==2.0.1"         # Add specific version
make add-dev PKG=pytest             # Add development dependency
make add-dev PKG="pytest==7.4.0"   # Add specific version for dev

# Install optional feature groups (installs to workspace .venv)
make install-cv                     # Computer vision support
make install-ocr                    # OCR capabilities
make install-video                  # Video processing
make install-web                    # Web scraping
make install-full                   # All optional features

# Add to specific feature groups
make add-cv PKG="opencv-python==4.8.0"    # Add CV package with version
make add-ocr PKG="paddleocr==2.7.0"       # Add OCR package with version

# Manage packages (modifies local pyproject.toml)
make remove PKG=outdated-package    # Remove package
make update                         # Update all dependencies
make update-pkg PKG=torch           # Update specific package
make show PKG=torch                 # Show package info
```

**Note**: This workspace uses its own isolated virtual environment. Dependencies are managed in the local `pyproject.toml` and installed to `./rag/.venv`.

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

## ⚙️ Configuration

### Environment Setup

Create a `.env` file from the example:

```bash
cp example.env .env
```

Configure key settings:

```bash
# Document processing
INPUT_DIR=/path/to/documents
OUTPUT_DIR=/path/to/processed
CACHE_DIR=/path/to/cache

# ML Models
EMBEDDING_MODEL=BAAI/bge-m3
DEVICE=cuda  # or cpu, mps

# Services
MINERU_SERVICE_URL=http://localhost:8001
NOUGAT_SERVICE_URL=http://localhost:8002

# OCR Settings
OCR_ENABLED=true
OCR_PROVIDER=paddleocr  # or nougat, mineru
```

## 🔧 Document Conversion

### Supported Formats

| Format   | Converter               | Features                                 |
| -------- | ----------------------- | ---------------------------------------- |
| PDF      | `pdf_converter.py`      | OCR, layout detection, table extraction  |
| Markdown | `md_converter.py`       | Syntax parsing, metadata extraction      |
| HTML     | `html_converter.py`     | Clean text extraction, link preservation |
| Python   | `python_converter.py`   | Code parsing, docstring extraction       |
| Jupyter  | `notebook_converter.py` | Cell processing, output handling         |
| Video    | `video_converter.py`    | Transcript extraction, scene detection   |

### Conversion Pipeline

```python
from file_conversion_router.api import convert_documents

# Basic conversion
results = convert_documents(
    input_path="/path/to/documents",
    output_path="/path/to/output",
    formats=["pdf", "md"]
)

# Advanced conversion with options
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

## 🧠 Embedding Generation

### Models and Strategies

The pipeline supports multiple embedding approaches:

```python
# Available embedding models
MODELS = {
    "bge-m3": "BAAI/bge-m3",           # Multi-modal (recommended)
    "bge-large": "BAAI/bge-large-en",  # English optimized
    "sentence-t5": "sentence-transformers/sentence-t5-base",
    "openai": "text-embedding-ada-002"  # API-based
}

# Chunking strategies
STRATEGIES = {
    "recursive": "Recursive character splitting",
    "paragraph": "Paragraph-based chunks",
    "sentence": "Sentence-based chunks",
    "semantic": "Semantic similarity splitting"
}
```

### Embedding Pipeline

```python
from file_conversion_router.embedding.embedding_create import create_embeddings

# Generate embeddings
embeddings = create_embeddings(
    documents_path="/path/to/processed",
    model="bge-m3",
    chunk_size=512,
    overlap=50,
    output_format="sqlite"  # or "pickle"
)
```

### Optimization Pipeline

The embedding optimization system allows for systematic testing of different configurations:

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

## 🌐 Service Integration

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

## 🧪 Testing

### Test Structure

```
tests/
├── test_file_conversion_router/    # Core conversion tests
├── test_services/                  # Service integration tests
├── test_embedding_optimization/    # Optimization pipeline tests
└── data/                          # Test data and fixtures
```

### Running Tests

```bash
# All tests (excluding slow ones)
make test

# Specific test categories
poetry run pytest tests/test_file_conversion_router/
poetry run pytest -m "not slow and not ocr and not gpu"

# Integration tests
poetry run pytest tests/test_services/ -v

# Performance tests
poetry run pytest -m "slow" --durations=10
```

### Test Markers

- `slow`: Time-intensive tests
- `cv`: Computer vision tests
- `ocr`: OCR functionality tests
- `gpu`: GPU-required tests
- `integration`: Service integration tests
- `unit`: Unit tests

## 🔒 Security & Performance

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

## 📊 Monitoring & Debugging

### Logging Configuration

```python
import logging
from file_conversion_router.utils.logger import setup_logger

# Setup structured logging
logger = setup_logger(
    name="rag_pipeline",
    level=logging.DEBUG,
    format="json"  # or "text"
)
```

### Performance Metrics

```bash
# Monitor conversion performance
poetry run python file_conversion_router/utils/time_measure.py --input /docs

# Hardware detection
poetry run python file_conversion_router/utils/hardware_detection.py
```

## 🚀 Deployment

### Docker Support

```bash
# Build RAG pipeline container
docker build -t tai-rag-pipeline .

# Run with GPU support
docker run --gpus all -v /data:/app/data tai-rag-pipeline
```

### Production Configuration

```bash
# Optimize for production
export TORCH_THREADS=4
export OMP_NUM_THREADS=4
export CUDA_VISIBLE_DEVICES=0

# Start processing service
poetry run uvicorn file_conversion_router.api:app --host 0.0.0.0 --port 8000
```

## 🔧 Troubleshooting

### Common Issues

**OCR Dependencies**

```bash
# Install OCR dependencies
make install-ocr

# Test OCR installation
poetry run python -c "import paddleocr; print('OCR ready')"
```

**GPU Memory Issues**

```bash
# Check GPU status
poetry run python -c "import torch; print(torch.cuda.get_device_properties(0))"

# Reduce batch size in config
export BATCH_SIZE=1
export MAX_LENGTH=512
```

**Model Download Issues**

```bash
# Clear model cache
rm -rf ~/.cache/huggingface/

# Download models manually
poetry run python -c "from transformers import AutoModel; AutoModel.from_pretrained('BAAI/bge-m3')"
```

### Debug Mode

```bash
# Enable debug logging
export LOG_LEVEL=DEBUG

# Run with profiling
poetry run python -m cProfile file_conversion_router/api.py
```

## 🤝 Contributing

1. Follow the existing architecture patterns
2. Add tests for new converters and services
3. Update documentation for API changes
4. Use type hints and docstrings
5. Ensure compatibility with all supported formats

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

## 📚 Additional Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [PyTorch Documentation](https://pytorch.org/docs/)
- [BGE Embedding Models](https://huggingface.co/BAAI)
- [TAI Project Documentation](../README.md)

## 🆘 Support

For issues and questions:

1. Check the troubleshooting section above
2. Review test cases for usage examples
3. Consult the main [TAI Documentation](../README.md)
4. Create an issue with detailed error logs
