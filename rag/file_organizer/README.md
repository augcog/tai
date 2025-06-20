# File Organizer

A modular system for organizing educational course files using LLMs for summarization, topic classification, and function classification.

## Features
- Summarizes course materials
- Classifies files by topic and function
- generates new reorganized folder based on classification
- Supports multiple LLM backends (OpenAI, NVIDIA, local HuggingFace, mock)

## Getting Started

### 1. Install Dependencies
Install Python dependencies (see `requirements.txt`). You may need to install additional packages for specific models (e.g., `openai`, `transformers`).

### 2. Configuration Files

#### `config.yaml`
This is the main configuration file for the organizer. It specifies which LLM profiles to use for summarization, topic classification, and function classification. Example:

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

#### `models.yaml`
Defines available LLM profiles and their parameters. This file is usually fixed and rarely changed. Example:

```yaml
profiles:
  local:
    provider: local_hf
    model_id: meta-llama/Meta-Llama-3-8B-Instruct
    quantization: 4bit
    max_new_tokens: 1024
    temperature: 0.7
  accurate:
    provider: openai
    model_name: gpt-4o
    temperature: 0.2
  mock:
    provider: mock
  nvidia_llama:
    provider: nvidia
    model_name: meta/llama3-8b-instruct
    temperature: 0.5
    max_new_tokens: 1024
```

#### Course-Specific Config (`course_config.yaml`)
For each course you want to organize, create a course-specific config file. This file can include:
- Predefined function classification (e.g., which folders are "practice", "study", etc.)
- Topic definitions for the course

Example:
```yaml
predefined_classification:
  practice: ["labs", "homework", "projects"]
  study: ["lectures", "notes"]
topic_summaries:
  Lecture1 Welcome: "provides an overview of the CS 61A course structure and expectations"
  Lecture2 Functions: "Content unavailable due to exceeding model's token limit."
  Lecture3 Control: "Examines control structures in Python, such as conditionals, loops, and environment management."
  Lecture4 Higher-Order Functions: "Explores control expressions, short-circuiting, and functions as return values."
  Lecture5 Environments: "Explores environments, higher-order functions, lambda expressions, and variable scoping."
```

### 3. Prompt Store
Prompt templates for LLMs are stored in the `src/services/prompt_store/` directory. You can customize these prompts to change how the models are instructed for summarization, classification, etc.

### 4. Running the Organizer
The main entry point is `src/organizer.py`. You can use the `FileOrganizer` class to run the full pipeline or just parts of it. Example usage:

```python
import asyncio
from src.config.config import OrganizerCfg
from src.organizer import FileOrganizer

async def main():
    organizer = FileOrganizer("course_config.yaml")
    await organizer.organize_course("/path/to/course_folder", "/path/to/output_folder")

if __name__ == "__main__":
    asyncio.run(main())
```

**Note:** The `organize_course` function will create `functions.json`, `topics.json`, and `summaries.json` in the input folder (the course folder you provide). These files contain the results of function classification, topic classification, and summarization, respectively.

### 5. Environment Variables
Some models require API keys. Set them in a `.env` file or as environment variables:
```
NVIDIA_API_KEY=your-nvidia-api-key
OPENAI_API_KEY=your-openai-api-key
```

### 6. Available Models
- **mock**: For testing, does not use any real LLM
- **openai**: Uses OpenAI's GPT models (e.g., gpt-3.5-turbo, gpt-4o)
- **nvidia**: Uses NVIDIA's OpenAI-compatible API
- **local_hf**: Loads a local HuggingFace model (currently untested)

You can select which model to use for each task in `config.yaml` by setting the `llm_profile`.

## Notes
- `config.yaml` and `models.yaml` are usually fixed for your deployment.
- Each course should have its own `course_config.yaml` for custom classification and topics.
- The local HuggingFace model option is present but currently untested.
- Prompts can be customized in the prompt store directory.

## License
MIT 