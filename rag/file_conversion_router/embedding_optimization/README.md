# Embedding Optimization

A flexible pipeline for processing markdown content and text chunks using LLMs (Large Language Models). This module is designed to enhance and optimize content through configurable processing tasks.

## Overview

The embedding optimizer provides two main functionalities:
- **Markdown Processing**: Process entire markdown documents
- **Chunk Processing**: Process individual text chunks

Perfect for:
- Content enhancement
- Text summarization
- Format standardization
- Custom text processing pipelines

## Quick Start

```python
from rag.file_conversion_router.classes.chunk import Chunk
from rag.file_conversion_router.embedding_optimization.src.pipeline.optimizer import EmbeddingOptimizer

# Initialize optimizer with your config
optimizer = EmbeddingOptimizer("rag/file_conversion_router/embedding_optimization/src/configs/default_config.yaml")

# Process markdown
result = optimizer.process_markdown("# Your markdown content")

# Process chunks
chunks = [Chunk(content="Your text")]
processed_chunks = optimizer.process_chunks(chunks)
```

## Pipeline Configuration

Example configuration:
[default_config.yaml](src/configs/default_config.yaml)



## Configuration Path

Default config location: `rag/file_conversion_router/embedding_optimization/config/default_config.yaml`

## Features

- 🔄 Flexible task pipeline
- 🔧 Configurable processing tasks
- 📝 Markdown and chunk support
- 🤖 Multiple model types support
- 🛠️ Error handling

## Models Supported

- Server-based models
- Local models
- Mock models (for testing)

## Error Handling

The optimizer provides detailed error information in processing results:
- Processing status
- Error messages
- Original content preservation

## Need Help?

See our tests for usage examples (under `tests/test_rag/test_file_conversion_router` directory).
