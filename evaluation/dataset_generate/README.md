# TAI Evaluation Tools

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![Poetry](https://img.shields.io/badge/poetry-1.4+-blue.svg)](https://python-poetry.org/)
[![OpenAI](https://img.shields.io/badge/OpenAI-API-green.svg)](https://openai.com/)

Dataset generation and evaluation tools for TAI (Teaching Assistant Intelligence) system performance analysis and quality assessment.

## 🚀 Quick Start

```bash
# Install dependencies
make install

# Generate evaluation dataset
make generate

# Analyze results
make analyze
```

## 🎯 Features

- **Dataset Generation**: Automated creation of evaluation datasets from course materials
- **Performance Analysis**: Comprehensive RAG system performance evaluation
- **Visualization**: Interactive charts and diagrams for result analysis
- **Multi-format Support**: Works with various course content formats
- **Statistical Analysis**: Detailed metrics and performance indicators

## 🏗️ Architecture

```
evaluation/dataset_generate/
├── src/                    # Source code (when created)
├── generate.py            # Dataset generation script
├── analyze.py             # Performance analysis script
├── output/                # Generated datasets and results
├── config/                # Configuration files
└── templates/             # Evaluation templates
```

## 🛠️ Development Commands

### Installation & Setup

```bash
make install              # Install core dependencies
make clean                # Clean build artifacts
```

### Dataset Operations

```bash
make generate             # Generate evaluation datasets
make analyze              # Analyze system performance
make visualize            # Create result visualizations
```

### Development & Testing

```bash
make test                # Run tests
make lint                # Run linting
make format              # Format code
```

## 📦 Package Management

The Evaluation Tools use the unified monorepo Poetry environment. All package management commands automatically modify the root `pyproject.toml`:

```bash
# Add new packages (adds to root monorepo)
make add PKG=openai                  # Add production dependency to root
make add-dev PKG=pytest             # Add development dependency to root

# Installation (always installs to root .venv)
make install                         # Install all dependencies to root

# Manage packages (all modify root pyproject.toml)
make remove PKG=torch                # Remove package from root
make update                          # Update all dependencies in root
make show                            # Show dependencies from root
```

**Note**: This component uses the unified monorepo environment. All dependencies are managed in the root `pyproject.toml` and installed to `/tai/.venv`. The local `pyproject.toml` serves as documentation of evaluation-specific dependencies.

## ⚙️ Configuration

### Environment Setup

Create and configure your environment:

```bash
# Create environment file
cp .env.example .env

# Configure API keys
OPENAI_API_KEY=your-openai-api-key

# Optional: Configure paths
RAG_DATA_PATH=/path/to/rag/data
COURSE_DATA_PATH=/path/to/course/materials
OUTPUT_PATH=/path/to/evaluation/output
```

### Evaluation Configuration

Create evaluation configuration files:

```yaml
# evaluation_config.yaml
dataset:
  name: "CS61A_Fall2024"
  course_code: "CS61A"
  sample_size: 100
  difficulty_levels: ["easy", "medium", "hard"]

generation:
  openai_model: "gpt-4o"
  temperature: 0.7
  max_tokens: 1000
  questions_per_topic: 10

evaluation:
  rag_model: "bge-m3"
  retrieval_k: 5
  similarity_threshold: 0.7
  metrics: ["accuracy", "precision", "recall", "f1"]
```

## 🔧 Usage

### Dataset Generation

Generate evaluation datasets from course materials:

```bash
# Basic generation
poetry run python generate.py --course CS61A --output datasets/

# With custom configuration
poetry run python generate.py \
  --config evaluation_config.yaml \
  --course CS61A \
  --topics "functions,recursion,oop" \
  --difficulty medium \
  --size 50

# Generate from specific course materials
poetry run generate-dataset \
  --input /path/to/course/materials \
  --output /path/to/evaluation/dataset \
  --format json
```

### Performance Analysis

Analyze RAG system performance:

```bash
# Basic analysis
poetry run python analyze.py --dataset datasets/cs61a_eval.json

# Comprehensive analysis with visualization
poetry run analyze-performance \
  --dataset datasets/cs61a_eval.json \
  --rag-data /path/to/rag/embeddings \
  --output analysis_results/ \
  --visualize

# Compare multiple models
poetry run analyze-performance \
  --dataset datasets/cs61a_eval.json \
  --models "bge-m3,openai-ada,sentence-t5" \
  --output comparison_results/
```

### Python API

```python
from src.generate import DatasetGenerator
from src.analyze import PerformanceAnalyzer

# Generate evaluation dataset
generator = DatasetGenerator(
    course_path="/path/to/course/materials",
    config_path="evaluation_config.yaml"
)

dataset = generator.generate_dataset(
    topics=["functions", "recursion", "data_structures"],
    num_questions=50,
    difficulty="medium"
)

# Analyze RAG performance
analyzer = PerformanceAnalyzer(
    rag_data_path="/path/to/rag/embeddings",
    dataset=dataset
)

results = analyzer.evaluate_performance(
    metrics=["accuracy", "precision", "recall"],
    k_values=[1, 3, 5, 10]
)

print(f"Average accuracy: {results['accuracy']:.3f}")
print(f"Top-5 recall: {results['recall@5']:.3f}")
```

## 📊 Dataset Generation

### Question Types

The system generates various types of evaluation questions:

| Type            | Description                  | Example                                                  |
| --------------- | ---------------------------- | -------------------------------------------------------- |
| **Factual**     | Direct information retrieval | "What is the time complexity of quicksort?"              |
| **Conceptual**  | Understanding of concepts    | "Explain the difference between iteration and recursion" |
| **Application** | Practical problem-solving    | "How would you implement a binary search tree?"          |
| **Analysis**    | Critical thinking            | "Compare the advantages of different sorting algorithms" |

### Difficulty Levels

- **Easy**: Basic recall and simple application
- **Medium**: Moderate reasoning and multi-step problems
- **Hard**: Complex analysis and synthesis

### Output Formats

Generated datasets include:

```json
{
  "metadata": {
    "course": "CS61A",
    "generated_at": "2024-01-15T10:30:00Z",
    "total_questions": 100,
    "topics": ["functions", "recursion", "oop"]
  },
  "questions": [
    {
      "id": "q001",
      "question": "What is the purpose of the __init__ method in Python?",
      "topic": "oop",
      "difficulty": "easy",
      "expected_answer": "The __init__ method is a constructor...",
      "source_material": "lecture_10_oop.md",
      "keywords": ["constructor", "initialization", "object"]
    }
  ]
}
```

## 📈 Performance Analysis

### Evaluation Metrics

The analyzer computes comprehensive performance metrics:

#### Retrieval Metrics

- **Recall@K**: Fraction of relevant documents retrieved in top-K results
- **Precision@K**: Fraction of retrieved documents that are relevant
- **MRR (Mean Reciprocal Rank)**: Average reciprocal rank of first relevant result
- **NDCG@K**: Normalized Discounted Cumulative Gain

#### Answer Quality Metrics

- **Semantic Similarity**: Cosine similarity between generated and expected answers
- **BLEU Score**: N-gram overlap between answers
- **ROUGE Score**: Recall-oriented metrics for answer quality
- **BERTScore**: Contextual embedding-based similarity

#### System Performance

- **Response Time**: Average time per query
- **Throughput**: Queries processed per second
- **Memory Usage**: Peak memory consumption
- **Error Rate**: Percentage of failed queries

### Analysis Reports

The analyzer generates detailed reports:

```python
# Performance summary
{
  "overall_performance": {
    "accuracy": 0.847,
    "avg_response_time": 1.23,
    "total_queries": 100
  },
  "by_difficulty": {
    "easy": {"accuracy": 0.92, "avg_time": 0.98},
    "medium": {"accuracy": 0.84, "avg_time": 1.15},
    "hard": {"accuracy": 0.76, "avg_time": 1.67}
  },
  "by_topic": {
    "functions": {"accuracy": 0.89, "coverage": 0.95},
    "recursion": {"accuracy": 0.82, "coverage": 0.88},
    "oop": {"accuracy": 0.81, "coverage": 0.92}
  }
}
```

## 📊 Visualization

### Available Visualizations

Generate insightful charts and diagrams:

```bash
# Performance over time
poetry run visualize-results --type timeline --data results.json

# Topic-wise performance breakdown
poetry run visualize-results --type heatmap --breakdown topic

# Difficulty distribution analysis
poetry run visualize-results --type distribution --metric accuracy

# Comparative model performance
poetry run visualize-results --type comparison --models "bge-m3,openai"
```

### Chart Types

- **Performance Timeline**: Track improvements over iterations
- **Topic Heatmaps**: Performance across different course topics
- **Distribution Charts**: Question difficulty and performance distributions
- **Comparison Plots**: Multi-model performance comparisons
- **Error Analysis**: Common failure patterns and error categories

## 🧪 Testing

### Test Structure

```
tests/
├── test_generation/        # Dataset generation tests
├── test_analysis/          # Performance analysis tests
├── test_visualization/     # Visualization tests
└── fixtures/               # Test data and fixtures
```

### Running Tests

```bash
# All tests
make test

# Specific test categories
poetry run pytest tests/test_generation/ -v
poetry run pytest tests/test_analysis/ -v

# With OpenAI API tests (requires API key)
poetry run pytest -m "openai" --api-key=$OPENAI_API_KEY

# Mock tests only (no API calls)
poetry run pytest -m "not openai and not api"
```

## 🔒 Security & Privacy

### Data Handling

- **Local Storage**: All generated datasets stored locally
- **API Safety**: Secure handling of OpenAI API keys
- **Content Privacy**: Course materials processed securely
- **Anonymization**: Option to anonymize student data in evaluations

### API Security

- **Rate Limiting**: Built-in OpenAI API rate limiting
- **Error Handling**: Robust error handling for API failures
- **Retry Logic**: Intelligent retry mechanisms for failed requests

## 📈 Performance Optimization

### Generation Optimization

```bash
# Batch processing for large datasets
export BATCH_SIZE=10
export MAX_WORKERS=4

# Optimize for speed vs quality
export GENERATION_MODE=fast  # or 'quality'
export OPENAI_MODEL=gpt-3.5-turbo  # faster, cheaper

# Cache frequently used prompts
export ENABLE_PROMPT_CACHE=true
```

### Analysis Optimization

```bash
# Parallel processing for large evaluations
export ANALYSIS_WORKERS=8

# Memory optimization for large datasets
export CHUNK_SIZE=1000
export ENABLE_STREAMING=true

# GPU acceleration for embedding models
export USE_GPU=true
export TORCH_DEVICE=cuda
```

## 🔧 Troubleshooting

### Common Issues

**OpenAI API Issues:**

```bash
# Test API connectivity
poetry run python -c "
import openai
client = openai.OpenAI()
print('API connection successful')
"

# Check API key
echo $OPENAI_API_KEY

# Handle rate limiting
export OPENAI_RATE_LIMIT=60  # requests per minute
```

**Memory Issues:**

```bash
# Reduce batch size
export BATCH_SIZE=5

# Enable streaming processing
export STREAM_MODE=true

# Use smaller models
export EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2
```

**Dataset Issues:**

```bash
# Validate dataset format
poetry run python -c "
import json
with open('dataset.json') as f:
    data = json.load(f)
print(f'Dataset has {len(data[\"questions\"])} questions')
"

# Check for required fields
poetry run python scripts/validate_dataset.py --dataset dataset.json
```

## 🤝 Contributing

### Adding New Evaluation Metrics

1. Create metric class in `src/metrics.py`:

```python
class NewMetric(BaseMetric):
    def compute(self, predictions, ground_truth):
        # Implementation here
        return score
```

2. Register in analyzer:

```python
AVAILABLE_METRICS["new_metric"] = NewMetric
```

### Adding New Question Types

```python
class NewQuestionGenerator(BaseQuestionGenerator):
    def generate_questions(self, content, num_questions):
        # Implementation here
        return questions
```

## 📚 Additional Resources

- [OpenAI API Documentation](https://platform.openai.com/docs)
- [Plotly Visualization Guide](https://plotly.com/python/)
- [RAG Evaluation Best Practices](https://arxiv.org/abs/2312.10003)
- [TAI Project Documentation](../../README.md)

## 🆘 Support

For issues and questions:

1. Check the troubleshooting section above
2. Verify API keys and configuration
3. Test with sample datasets first
4. Consult the main [TAI Documentation](../../README.md)

## 📋 Sample Workflows

### Complete Evaluation Pipeline

```bash
# 1. Generate evaluation dataset
poetry run python generate.py \
  --course CS61A \
  --size 100 \
  --output datasets/cs61a_eval.json

# 2. Run RAG evaluation
poetry run python analyze.py \
  --dataset datasets/cs61a_eval.json \
  --rag-data /path/to/rag/embeddings \
  --output results/

# 3. Generate visualizations
poetry run python visualize.py \
  --results results/analysis.json \
  --output visualizations/

# 4. Generate report
poetry run python report.py \
  --analysis results/analysis.json \
  --visualizations visualizations/ \
  --output final_report.html
```

This evaluation toolkit provides comprehensive assessment capabilities for the TAI system, enabling continuous improvement and quality assurance.
