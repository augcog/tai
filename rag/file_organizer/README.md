# TAI File Organizer

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![Poetry](https://img.shields.io/badge/poetry-1.4+-blue.svg)](https://python-poetry.org/)
[![Transformers](https://img.shields.io/badge/transformers-4.30+-orange.svg)](https://huggingface.co/transformers/)

ML-based file organization and classification utility for educational course materials using LLMs for intelligent content analysis.

## 🚀 Quick Start

```bash
# Install dependencies
make install

# Install dependencies from unified monorepo
make install

# Run file organization
make organize
```

## 🎯 Features

- **Smart Summarization**: AI-powered content summarization of course materials
- **Topic Classification**: Automatic categorization by subject and theme
- **Function Classification**: Identifies material purpose (practice, study, reference)
- **Multi-LLM Support**: OpenAI, NVIDIA, local HuggingFace, and mock backends
- **Course-Specific Configuration**: Customizable classification rules per course
- **Batch Processing**: Efficient organization of large course directories

## 🏗️ Architecture

```
file_organizer/
├── src/
│   ├── core/                    # Core processing modules
│   │   ├── file_organizer.py   # Main organizer class
│   │   ├── summarizer.py       # Content summarization
│   │   ├── topic_classifier.py # Topic classification
│   │   └── func_classifier.py  # Function classification
│   ├── services/               # External service integrations
│   │   ├── models.py          # LLM model management
│   │   └── prompt_service.py  # Prompt templates
│   ├── config/                # Configuration management
│   └── utils/                 # Utilities and helpers
├── config.yaml               # Main configuration
├── models.yaml              # LLM model definitions
└── course_config_sample.yaml # Sample course configuration
```

## 🛠️ Development Commands

### Installation & Setup

```bash
make install              # Install dependencies from root monorepo
make clean                # Clean build artifacts
```

### File Organization

```bash
make organize             # Run file organization pipeline
make classify             # Run classification only
make summarize           # Run summarization only
```

### Development & Testing

```bash
make test                # Run tests
make lint                # Run linting
make format              # Format code
make dev                 # Start development mode
```

## 📦 Package Management

The File Organizer uses the unified monorepo Poetry environment. All package management commands automatically modify the root `pyproject.toml`:

```bash
# Add new packages (adds to root monorepo)
make add PKG=torch                    # Add production dependency to root
make add-dev PKG=pytest              # Add development dependency to root

# Installation (always installs to root .venv)
make install                         # Install all dependencies to root

# Manage packages (all modify root pyproject.toml)
make remove PKG=torch                 # Remove package from root
make update                          # Update all dependencies in root
make show                            # Show dependencies from root
```

**Note**: This component uses the unified monorepo environment. All dependencies are managed in the root `pyproject.toml` and installed to `/tai/.venv`. The local `pyproject.toml` serves as documentation of file organizer-specific dependencies.

## ⚙️ Configuration

### 1. Main Configuration (`config.yaml`)

Controls the organizer behavior and LLM selection:

```yaml
summarizer:
  max_length: 300
  llm_profile: nvidia_llama

topic_classifier:
  max_topic_num: 3
  llm_profile: nvidia_llama

func_classifier:
  llm_profile: nvidia_llama
```

### 2. Model Definitions (`models.yaml`)

Defines available LLM profiles and parameters:

```yaml
profiles:
  # Local HuggingFace model with quantization
  local:
    provider: local_hf
    model_id: meta-llama/Meta-Llama-3-8B-Instruct
    quantization: 4bit
    max_new_tokens: 1024
    temperature: 0.7

  # OpenAI GPT models
  accurate:
    provider: openai
    model_name: gpt-4o
    temperature: 0.2

  # NVIDIA API
  nvidia_llama:
    provider: nvidia
    model_name: meta/llama3-8b-instruct
    temperature: 0.5
    max_new_tokens: 1024

  # Mock for testing
  mock:
    provider: mock
```

### 3. Course-Specific Configuration

Create custom configurations for each course:

```yaml
# course_config_cs61a.yaml
predefined_classification:
  practice: ["labs", "homework", "projects", "assignments"]
  study: ["lectures", "notes", "readings"]
  reference: ["guides", "documentation", "resources"]

topic_summaries:
  "Lecture 1 - Welcome": "Course overview and expectations"
  "Lecture 2 - Functions": "Python functions and control flow"
  "Lecture 3 - Control": "Conditionals, loops, and environments"
  "Lecture 4 - Higher-Order Functions": "Functions as first-class objects"

custom_topics:
  - "Recursion and Tree Structures"
  - "Object-Oriented Programming"
  - "Interpreters and Language Processing"
```

### 4. Environment Setup

Set up API keys and configuration:

```bash
# Create environment file
cp .env.example .env

# Configure API keys
NVIDIA_API_KEY=your-nvidia-api-key
OPENAI_API_KEY=your-openai-api-key

# Optional: Configure local model paths
HUGGINGFACE_CACHE_DIR=/path/to/model/cache
TORCH_HOME=/path/to/torch/models
```

## 🔧 Usage

### Command Line Interface

```bash
# Basic organization
poetry run file-organizer --input /path/to/course --output /path/to/organized

# With specific configuration
poetry run organize-files \
  --input /course/cs61a \
  --output /organized/cs61a \
  --config course_config_cs61a.yaml \
  --model-profile accurate

# Classification only
poetry run organize-files --input /course --classify-only --output results.json
```

### Python API

```python
import asyncio
from src.organizer import FileOrganizer

async def organize_course():
    # Initialize organizer with course-specific config
    organizer = FileOrganizer("course_config_cs61a.yaml")

    # Run full organization pipeline
    results = await organizer.organize_course(
        input_path="/path/to/cs61a_materials",
        output_path="/path/to/organized_cs61a"
    )

    print(f"Organized {results['files_processed']} files")
    print(f"Topics found: {results['topics']}")
    print(f"Functions classified: {results['functions']}")

# Run the organizer
asyncio.run(organize_course())
```

### Advanced Usage

```python
from src.core.summarizer import Summarizer
from src.core.topic_classifier import TopicClassifier
from src.core.func_classifier import FuncClassifier

# Use individual components
summarizer = Summarizer(config_path="config.yaml")
topic_classifier = TopicClassifier(config_path="config.yaml")
func_classifier = FuncClassifier(config_path="config.yaml")

# Process individual files
summary = await summarizer.summarize_file("/path/to/document.pdf")
topics = await topic_classifier.classify_file("/path/to/document.pdf")
function = await func_classifier.classify_file("/path/to/document.pdf")
```

## 🧠 LLM Backends

### Supported Providers

| Provider     | Description              | Requirements    | Performance                  |
| ------------ | ------------------------ | --------------- | ---------------------------- |
| **OpenAI**   | GPT-3.5/GPT-4 models     | API key         | High accuracy, fast          |
| **NVIDIA**   | NVIDIA NIM API           | API key         | Good balance, cost-effective |
| **Local HF** | Local HuggingFace models | GPU recommended | Private, customizable        |
| **Mock**     | Testing provider         | None            | Testing only                 |

### Provider-Specific Setup

**OpenAI:**

```bash
export OPENAI_API_KEY=your-key
# Use profiles: accurate, fast, balanced
```

**NVIDIA:**

```bash
export NVIDIA_API_KEY=your-key
# Use profiles: nvidia_llama, nvidia_mixtral
```

**Local HuggingFace:**

```bash
# Install dependencies (GPU support included in unified environment)
make install

# Configure in models.yaml
local_llama:
  provider: local_hf
  model_id: meta-llama/Meta-Llama-3-8B-Instruct
  quantization: 4bit
```

## 📊 Output Formats

The organizer generates several output files:

### 1. Organized Directory Structure

```
organized_course/
├── practice/
│   ├── labs/
│   ├── homework/
│   └── projects/
├── study/
│   ├── lectures/
│   ├── notes/
│   └── readings/
└── reference/
    ├── guides/
    └── documentation/
```

### 2. Classification Results (`functions.json`)

```json
{
  "lab01.py": {
    "function": "practice",
    "confidence": 0.95,
    "reasoning": "Contains programming exercises and solutions"
  },
  "lecture_notes.pdf": {
    "function": "study",
    "confidence": 0.88,
    "reasoning": "Educational content for learning"
  }
}
```

### 3. Topic Analysis (`topics.json`)

```json
{
  "topics": [
    {
      "name": "Recursion and Tree Structures",
      "files": ["tree_recursion.py", "recursion_lab.py"],
      "summary": "Covers recursive algorithms and tree data structures"
    }
  ]
}
```

### 4. Content Summaries (`summaries.json`)

```json
{
  "lecture01.pdf": {
    "summary": "Introduction to computer science concepts...",
    "key_topics": ["functions", "variables", "control flow"],
    "word_count": 1250
  }
}
```

## 🧪 Testing

### Test Structure

```
tests/
├── test_components/        # Individual component tests
├── test_model/            # LLM provider tests
└── test_organizer/        # Integration tests
```

### Running Tests

```bash
# All tests
make test

# Specific components
poetry run pytest tests/test_components/test_summarizer.py -v
poetry run pytest tests/test_model/ -v

# Skip API-dependent tests
poetry run pytest -m "not openai and not nvidia"

# Test with mock models only
poetry run pytest tests/ --mock-only
```

## 🔒 Security & Privacy

### Data Handling

- **Local Processing**: Option to run entirely locally with HuggingFace models
- **API Safety**: API keys stored securely in environment variables
- **Content Privacy**: Summaries and classifications stored locally
- **Temporary Files**: Automatic cleanup of processing artifacts

### Model Security

- **Quantization**: 4-bit/8-bit quantization for resource efficiency
- **Input Validation**: Comprehensive file type and content validation
- **Rate Limiting**: Built-in API rate limiting and retry logic

## 📈 Performance & Optimization

### Hardware Requirements

| Model Type | RAM  | GPU VRAM | Processing Speed          |
| ---------- | ---- | -------- | ------------------------- |
| Mock       | 1GB  | N/A      | Instant                   |
| OpenAI API | 2GB  | N/A      | Fast (network dependent)  |
| NVIDIA API | 2GB  | N/A      | Fast (network dependent)  |
| Local 7B   | 8GB  | 6GB+     | Medium                    |
| Local 13B+ | 16GB | 12GB+    | Slower but higher quality |

### Optimization Tips

```bash
# For large courses
export BATCH_SIZE=4
export MAX_WORKERS=2

# For limited resources
export USE_QUANTIZATION=true
export TORCH_CUDA_MEMORY_FRACTION=0.8

# For faster processing
export ENABLE_CACHING=true
export PARALLEL_PROCESSING=true
```

## 🔧 Troubleshooting

### Common Issues

**Model Loading Errors:**

```bash
# Check GPU availability
poetry run python -c "import torch; print(torch.cuda.is_available())"

# Clear model cache
rm -rf ~/.cache/huggingface/

# Test with mock model first
poetry run organize-files --input test_files --model-profile mock
```

**API Connection Issues:**

```bash
# Verify API keys
echo $OPENAI_API_KEY
echo $NVIDIA_API_KEY

# Test API connectivity
poetry run python -c "
from src.services.models import get_model
model = get_model('openai')
print('API connection successful')
"
```

**Memory Issues:**

```bash
# Use smaller models
export MODEL_SIZE=7b

# Enable quantization
export USE_4BIT_QUANTIZATION=true

# Reduce batch size
export BATCH_SIZE=1
```

## 🤝 Contributing

### Adding New LLM Providers

1. Create provider class in `src/services/models.py`:

```python
class NewProviderModel(BaseLLMModel):
    def __init__(self, config):
        self.config = config

    async def generate(self, prompt: str) -> str:
        # Implementation here
        pass
```

2. Register in model factory:

```python
MODEL_PROVIDERS["new_provider"] = NewProviderModel
```

3. Add configuration in `models.yaml`:

```yaml
new_profile:
  provider: new_provider
  model_name: model-id
  temperature: 0.7
```

### Customizing Prompts

Edit templates in `src/services/prompt_service.py`:

```python
SUMMARIZATION_PROMPT = """
Summarize the following educational content:
{content}

Focus on key learning objectives and main concepts.
"""
```

## 📚 Additional Resources

- [HuggingFace Transformers](https://huggingface.co/docs/transformers)
- [OpenAI API Documentation](https://platform.openai.com/docs)
- [NVIDIA NIM API](https://build.nvidia.com/explore/discover)
- [TAI Project Documentation](../../README.md)

## 🆘 Support

For issues and questions:

1. Check the troubleshooting section above
2. Test with mock models to isolate issues
3. Review configuration file examples
4. Consult the main [TAI Documentation](../../README.md)
