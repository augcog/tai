# Claude Evaluation Documentation

This file provides specialized guidance for working with the Evaluation component of the TAI monorepo.

## Component Overview

The Evaluation Pipeline provides comprehensive test suite and dataset generation capabilities to measure the reliability and accuracy of the TAI's underlying Retrieval-Augmented Generation (RAG) agent. Located in `/evaluation/`, it includes dataset generation tools and performance analysis frameworks.

## Architecture

### Core Technologies
- **Dataset Generation**: Automated evaluation dataset creation tailored to TAI
- **Analysis Tools**: Bias detection and relationship visualization using Sankey graphs
- **OpenAI Integration**: Uses OpenAI API for dataset generation
- **Performance Benchmarking**: Systematic evaluation of RAG system performance

### Evaluation Workflow
1. **Dataset Generation**: Create evaluation datasets from input data
2. **Bias Analysis**: Analyze generated datasets for statistical biases
3. **Visualization**: Generate Sankey graphs for relationship analysis
4. **Performance Testing**: Benchmarking and reliability measurements

## Key File Locations

### Core Modules
- `evaluation/dataset_generate/` - Dataset generation tools
- `evaluation/dataset_generate/src/` - Source code for generation logic
- `evaluation/dataset_generate/input/` - Input JSON files directory
- `evaluation/dataset_generate/output/` - Generated datasets directory

### Configuration Files
- `evaluation/dataset_generate/pyproject.toml` - Local dependencies (documentation)
- `evaluation/dataset_generate/requirements.txt` - Requirements export
- `evaluation/dataset_generate/Makefile` - Component-specific commands

### Scripts
- `evaluation/dataset_generate/generate.py` - Main dataset generation script
- `evaluation/dataset_generate/analyze.py` - Dataset analysis script

## Development Commands

**Note: Uses unified monorepo environment**

### Installation
```bash
# Install dependencies (from root)
make install
```

### Dataset Generation
```bash
# Generate evaluation dataset
python -m evaluation.dataset_generate.generate <input_filename> [--num_pairs] [--quiet]

# Example
python -m evaluation.dataset_generate.generate sample_input.json --num_pairs 50
```

### Dataset Analysis
```bash
# Analyze generated dataset for biases
python -m evaluation.dataset_analyze.analyze <input_filename> [--graph]

# Example with Sankey graph
python -m evaluation.dataset_analyze.analyze generated_dataset.json --graph
```

### Development & Testing
```bash
make dev                 # Start Jupyter notebook server
make test                # Run tests for evaluation tools
make test-no-api        # Run tests without OpenAI API calls
make lint               # Run linting on evaluation files only
make format             # Format evaluation code only
```

### Using Makefile (from evaluation/dataset_generate/)
```bash
make generate CONFIG=config.yaml    # Generate dataset with config
make analyze DATA=results.json      # Analyze results
make visualize DATA=results.json    # Create visualizations
```

## Environment Setup

### Required Environment Variables
```bash
# OpenAI API Key (required for dataset generation)
export OPENAI_API_KEY="your_api_key_here"
```

### Configuration Methods
1. Add to shell profile (`.bashrc`, `.zshrc`)
2. Create `.env` file in component directory
3. Set as environment variable before running commands

## Dataset Generation Workflow

### Input Data Preparation
1. **Place Input Files**: Put JSON files in `/evaluation/dataset_generate/input/`
2. **Configure Generation**: Specify number of pairs and options
3. **Run Generation**: Execute generation command with parameters
4. **Review Output**: Generated datasets saved in `/evaluation/dataset_generate/output/`

### Generation Parameters
- `<input_filename>`: Name of the input JSON file in input directory
- `--num_pairs`: (Optional) Specify the number of pairs to generate
- `--quiet`: (Optional) Suppress output logs for cleaner console

### Example Usage
```bash
# Basic generation
python -m evaluation.dataset_generate.generate course_data.json

# With specific number of pairs
python -m evaluation.dataset_generate.generate course_data.json --num_pairs 100

# Quiet mode
python -m evaluation.dataset_generate.generate course_data.json --num_pairs 50 --quiet
```

## Dataset Analysis Features

### Bias Detection
- **Statistical Analysis**: Identifies biases in generated datasets
- **Label Distribution**: Analyzes input and output dataset labels
- **Relationship Mapping**: Shows connections between data elements

### Visualization Tools
- **Sankey Graphs**: Visualize data flow and relationships
- **Statistical Charts**: Distribution and bias visualization
- **Export Options**: Save visualizations as images

### Analysis Parameters
- `<input_filename>`: Name of the dataset file to analyze
- `--graph`: Option to generate Sankey graph visualization

### Example Analysis
```bash
# Basic analysis
python -m evaluation.dataset_analyze.analyze evaluation_dataset.json

# With graph generation
python -m evaluation.dataset_analyze.analyze evaluation_dataset.json --graph
```

## Package Management

**Note: Uses unified monorepo environment - all changes modify root pyproject.toml**

```bash
# Add packages (modifies root pyproject.toml)
make add PKG=package-name        # Add production dependency to root
make add-dev PKG=package-name    # Add development dependency to root
make remove PKG=package-name     # Remove package from root
make update                      # Update all dependencies in root
make show                        # Show dependencies from root
```

## Directory Structure

```
evaluation/
├── dataset_generate/
│   ├── input/                    # Input JSON files
│   ├── output/                   # Generated datasets and analysis
│   ├── src/                      # Source code
│   ├── tests/                    # Test files
│   ├── generate.py              # Main generation script
│   ├── analyze.py               # Analysis script
│   ├── Makefile                 # Component commands
│   ├── pyproject.toml           # Local dependency documentation
│   └── requirements.txt         # Requirements export
└── README.md                    # Component documentation
```

## Output Files

### Generated Datasets
- **Location**: `/evaluation/dataset_generate/output/`
- **Format**: JSON files with evaluation data pairs
- **Naming**: Based on input filename with generation timestamp

### Analysis Results
- **Location**: `/evaluation/dataset_analyze/output/`
- **Types**: 
  - Statistical analysis JSON files
  - Sankey graph visualizations (PNG)
  - Bias detection reports

### Example Output Files
```
output/
├── evaluation_dataset_106b_ed.json      # Generated dataset
├── datasets_analysis_106b_ed.json       # Analysis results
└── category_flow_diagram_106b_ed.png    # Sankey visualization
```

## Testing Strategy

### Test Categories
- **Unit Tests**: Individual component testing
- **Integration Tests**: Full pipeline testing
- **API Tests**: OpenAI integration testing (marked with `@pytest.mark.openai`)
- **Performance Tests**: Generation speed and accuracy benchmarks

### Running Tests
```bash
# All tests
make test

# Exclude OpenAI API tests (useful for CI/CD without API keys)
make test-no-api

# From root directory (unified environment)
poetry run pytest evaluation/dataset_generate/tests/ -v
```

### Test Markers
- `openai`: Tests requiring OpenAI API access
- `slow`: Time-intensive generation tests
- `integration`: End-to-end pipeline tests

## Development Patterns

### Dataset Generation Best Practices
1. **Input Validation**: Ensure JSON input files are well-formed
2. **API Rate Limiting**: Respect OpenAI API rate limits
3. **Error Handling**: Graceful handling of API failures
4. **Progress Tracking**: Monitor generation progress for large datasets

### Analysis Workflow
1. **Data Quality Checks**: Validate generated datasets
2. **Bias Detection**: Systematic bias analysis
3. **Visualization**: Create clear, informative charts
4. **Report Generation**: Comprehensive analysis reports

### Configuration Management
- Use consistent naming conventions for input/output files
- Maintain configuration files for reproducible results
- Document generation parameters for each dataset

## Common Issues & Troubleshooting

### OpenAI API Issues
```bash
# Check API key configuration
echo $OPENAI_API_KEY

# Test API connection
python -c "import openai; print('API key configured')"
```

### Generation Failures
```bash
# Check input file format
python -c "import json; json.load(open('input/your_file.json'))"

# Verify output directory permissions
ls -la output/

# Run in debug mode (without --quiet)
python -m evaluation.dataset_generate.generate input.json
```

### Analysis Issues
```bash
# Verify dataset file exists
ls -la output/evaluation_dataset_*.json

# Check for required dependencies
make test-no-api
```

## Integration with TAI Components

### Backend Integration
- Generated datasets can be used to test backend RAG responses
- Evaluation results inform backend optimization decisions
- Performance benchmarks guide backend scaling

### RAG Pipeline Integration
- Datasets test document processing accuracy
- Embedding quality evaluation
- Retrieval performance measurement

### Continuous Evaluation
- Automated dataset generation for new course content
- Regular performance benchmarking
- Bias detection in production data

## Performance Considerations

### Generation Optimization
- **Batch Processing**: Generate multiple pairs efficiently
- **API Usage**: Optimize OpenAI API calls to minimize costs
- **Caching**: Cache intermediate results for repeated runs
- **Parallel Processing**: Utilize concurrent generation when possible

### Analysis Performance
- **Memory Usage**: Efficient handling of large datasets
- **Visualization**: Optimized graph generation for large data
- **Statistical Computation**: Fast bias detection algorithms