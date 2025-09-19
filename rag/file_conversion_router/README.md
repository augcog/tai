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

2. **Tree Structure** (`file.md.tree.txt`)
   - Hierarchical representation of content structure
   - Used for navigation and context understanding

3. **Page Chunks** (`file.pkl`)
   - Serialized Page objects with chunks
   - Token-aware splitting (optimized for ~400 tokens)
   - Maintains semantic coherence

4. **Database Entries**
   - File metadata in SQLite database
   - Chunk records with embeddings
   - Index mappings for efficient retrieval

### Usage Examples

#### Basic File Conversion

```python
from file_conversion_router.api import convert_directory

# Convert a single course directory
convert_directory("/path/to/course_config.yaml")
```

#### Batch Processing Multiple Courses

```python
from file_conversion_router.api import process_courses_from_master_config

# Process all courses marked for update
process_courses_from_master_config("/path/to/master_config.yaml")
```

#### Page Creation with Chunking

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

#### AI-Enhanced Processing (with OpenAI)

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

#### Database Merging

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

### Advanced Features

- **Caching Mechanism**: Avoids re-conversion of unchanged files
- **Performance Monitoring**: Logs and monitors conversion time for each file
- **Conversion Fidelity**: 95% similarity threshold for test validations
- **Robust Logging**: Detailed logging of conversion processes
- **Ignore Patterns**: Support for `.conversionignore` file to exclude files/folders

### Testing

Run tests from the parent RAG directory:

```bash
cd ..
make test               # Run all tests
make test-cv           # Test computer vision components
make test-ocr          # Test OCR functionality
```

For more detailed examples and expected outputs, refer to the test suite located at `tests/test_file_conversion_router/test_api.py`.