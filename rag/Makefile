.PHONY: help install install-basic install-cv install-ocr install-full
.PHONY: dev test lint format clean
.PHONY: add add-dev remove update show
.PHONY: convert embed process
.PHONY: test-cv test-ocr test-ml
.PHONY: setup-models download-models

POETRY_RUN = poetry run
PYTHON = $(POETRY_RUN) python
PYTEST = $(POETRY_RUN) pytest

# Default target
help: ## Show this help message
	@echo "RAG Pipeline - Document Processing & Embedding Generation"
	@echo "========================================================"
	@echo ""
	@echo "🚀 Quick Start:"
	@echo "  make install-basic    # Install core dependencies only"
	@echo "  make install-full     # Install everything (large download!)"
	@echo ""
	@echo "📦 Installation Options:"
	@awk 'BEGIN {FS = ":.*?## "} /^install.*:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""
	@echo "🛠️  Development Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(dev|test|lint|format|clean)"
	@echo ""
	@echo "🔄 Processing Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(convert|embed|process|models)"
	@echo ""
	@echo "📦 Package Management:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(add|remove|update|show)"
	@echo ""
	@echo "💡 Examples:"
	@echo "  make convert INPUT=docs/ OUTPUT=processed/"
	@echo "  make add PKG=torch GROUP=ml-heavy"
	@echo "  make install-cv          # Just computer vision"
	@echo ""

# Installation commands with different feature sets
install: install-basic ## Install basic dependencies (alias for install-basic)

install-basic: ## Install core dependencies only (fastest)
	@echo "📦 Installing basic RAG dependencies..."
	@echo "🔒 Creating isolated virtual environment..."
	poetry install
	@poetry env info --path | sed 's/^/🐍 Virtual environment: /'
	@echo "✅ Basic dependencies installed in isolated virtual environment!"
	@echo "💡 For more features: make install-cv, make install-ocr, make install-full"

install-cv: ## Install with computer vision support
	@echo "📦 Installing with computer vision support..."
	poetry install --extras cv
	@echo "✅ Computer vision dependencies installed!"

install-ocr: ## Install with OCR support (large download)
	@echo "📦 Installing with OCR support (this may take a while)..."
	poetry install --extras ocr
	@echo "✅ OCR dependencies installed!"

install-ml-heavy: ## Install heavy ML packages for performance
	@echo "📦 Installing heavy ML packages..."
	poetry install --extras ml-heavy
	@echo "✅ Heavy ML packages installed!"

install-nlp: ## Install NLP processing packages
	@echo "📦 Installing NLP packages..."
	poetry install --extras nlp
	@echo "✅ NLP packages installed!"

install-video: ## Install video processing support
	@echo "📦 Installing video processing support..."
	poetry install --extras video
	@echo "✅ Video processing packages installed!"

install-web: ## Install web scraping support
	@echo "📦 Installing web scraping support..."
	poetry install --extras web
	@echo "✅ Web scraping packages installed!"

install-full: ## Install all features (LARGE download - 5GB+)
	@echo "🚨 WARNING: This will download 5GB+ of dependencies!"
	@echo "📦 Installing ALL RAG features..."
	poetry install --extras full --with cv,ocr,ml-heavy,ml-training,scientific,nlp,video,web,formats
	@echo "✅ Full installation complete! All features available."

install-dev: ## Install with development dependencies
	@echo "📦 Installing with development dependencies..."
	poetry install --with dev
	@echo "✅ Development environment ready!"

# Development
dev: ## Start development server
	@echo "🚀 Starting RAG development server..."
	$(PYTHON) -m src.server

dev-api: ## Start FastAPI development server
	@echo "🚀 Starting RAG API server..."
	$(POETRY_RUN) uvicorn src.api:app --reload --port 8001

# Testing with feature-specific tests
test: ## Run basic tests
	@echo "🧪 Running basic tests..."
	$(PYTEST) -v -m "not slow and not cv and not ocr and not gpu"

test-all: ## Run all tests (requires full installation)
	@echo "🧪 Running all tests..."
	$(PYTEST) -v

test-unit: ## Run unit tests only
	@echo "🧪 Running unit tests..."
	$(PYTEST) -v -m "unit"

test-integration: ## Run integration tests
	@echo "🧪 Running integration tests..."
	$(PYTEST) -v -m "integration"

test-cv: ## Run computer vision tests (requires CV packages)
	@echo "🧪 Running computer vision tests..."
	$(PYTEST) -v -m "cv"

test-ocr: ## Run OCR tests (requires OCR packages)
	@echo "🧪 Running OCR tests..."
	$(PYTEST) -v -m "ocr"

test-ml: ## Run ML tests (requires ML packages)
	@echo "🧪 Running ML tests..."
	$(PYTEST) -v -m "not slow" --timeout=300

test-slow: ## Run slow/heavy tests
	@echo "🧪 Running slow tests (this may take a while)..."
	$(PYTEST) -v -m "slow" --timeout=1800

# Code quality
lint: ## Run linting checks
	@echo "🔍 Running linting checks..."
	$(POETRY_RUN) ruff check .
	$(POETRY_RUN) mypy src --ignore-missing-imports
	@echo "✅ Linting completed!"

format: ## Format code
	@echo "🎨 Formatting code..."
	$(POETRY_RUN) black .
	$(POETRY_RUN) ruff format .
	@echo "✅ Code formatted!"

# RAG Processing commands
convert: ## Convert files (use: make convert INPUT=path/to/input OUTPUT=path/to/output)
	@if [ -z "$(INPUT)" ] || [ -z "$(OUTPUT)" ]; then \
		echo "❌ Usage: make convert INPUT=input_path OUTPUT=output_path"; \
		echo "Example: make convert INPUT=docs/ OUTPUT=processed/"; \
		exit 1; \
	fi
	@echo "🔄 Converting files from $(INPUT) to $(OUTPUT)..."
	$(PYTHON) -m src.conversion.convert_files --input $(INPUT) --output $(OUTPUT)
	@echo "✅ File conversion completed!"

embed: ## Create embeddings (use: make embed INPUT=processed/ OUTPUT=embeddings/)
	@if [ -z "$(INPUT)" ] || [ -z "$(OUTPUT)" ]; then \
		echo "❌ Usage: make embed INPUT=processed_path OUTPUT=embeddings_path"; \
		echo "Example: make embed INPUT=processed/ OUTPUT=embeddings/"; \
		exit 1; \
	fi
	@echo "🧠 Creating embeddings from $(INPUT) to $(OUTPUT)..."
	$(PYTHON) -m src.embedding.create_embeddings --input $(INPUT) --output $(OUTPUT)
	@echo "✅ Embedding creation completed!"

process: ## Full pipeline (use: make process INPUT=docs/ OUTPUT=final/)
	@if [ -z "$(INPUT)" ] || [ -z "$(OUTPUT)" ]; then \
		echo "❌ Usage: make process INPUT=input_path OUTPUT=output_path"; \
		echo "Example: make process INPUT=docs/ OUTPUT=final/"; \
		exit 1; \
	fi
	@echo "🔄 Running full RAG pipeline..."
	@echo "Step 1: Converting files..."
	$(PYTHON) -m src.conversion.convert_files --input $(INPUT) --output $(OUTPUT)/processed
	@echo "Step 2: Creating embeddings..."
	$(PYTHON) -m src.embedding.create_embeddings --input $(OUTPUT)/processed --output $(OUTPUT)/embeddings
	@echo "✅ Full pipeline completed!"

# Model management
download-models: ## Download required models
	@echo "📥 Downloading required models..."
	$(PYTHON) -c "from src.models import download_base_models; download_base_models()"
	@echo "✅ Models downloaded!"

setup-models: ## Setup and verify models
	@echo "🔧 Setting up models..."
	$(PYTHON) -c "from src.models import setup_models; setup_models()"
	@echo "✅ Models setup completed!"

# Package management
add: ## Add production dependency (use: make add PKG=package-name [GROUP=group-name])
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add PKG=package-name [GROUP=group-name]"; exit 1; fi
	@if [ -n "$(GROUP)" ]; then \
		echo "📦 Adding $(PKG) to group $(GROUP)..."; \
		poetry add --group $(GROUP) $(PKG); \
	else \
		echo "📦 Adding package: $(PKG)"; \
		poetry add $(PKG); \
	fi
	@echo "✅ Package $(PKG) added!"

add-dev: ## Add development dependency (use: make add-dev PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add-dev PKG=package-name"; exit 1; fi
	@echo "📦 Adding development package: $(PKG)"
	poetry add --group dev $(PKG)
	@echo "✅ Development package $(PKG) added!"

add-optional: ## Add to optional group (use: make add-optional PKG=package GROUP=cv)
	@if [ -z "$(PKG)" ] || [ -z "$(GROUP)" ]; then \
		echo "❌ Usage: make add-optional PKG=package-name GROUP=group-name"; \
		echo "Available groups: cv, ocr, ml-heavy, nlp, video, web, formats"; \
		exit 1; \
	fi
	@echo "📦 Adding $(PKG) to optional group $(GROUP)..."
	poetry add --group $(GROUP) $(PKG)
	@echo "✅ Package $(PKG) added to $(GROUP) group!"

remove: ## Remove dependency (use: make remove PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make remove PKG=package-name"; exit 1; fi
	@echo "📦 Removing package: $(PKG)"
	poetry remove $(PKG)
	@echo "✅ Package $(PKG) removed!"

update: ## Update all dependencies
	@echo "⬆️  Updating dependencies..."
	poetry update
	@echo "✅ Dependencies updated!"

show: ## Show dependency information
	@if [ -z "$(PKG)" ]; then \
		echo "📊 Showing all dependencies:"; \
		poetry show; \
	else \
		echo "📊 Showing package info: $(PKG)"; \
		poetry show $(PKG); \
	fi

show-groups: ## Show available dependency groups
	@echo "📊 Available dependency groups:"
	@echo "  Core: Always installed"
	@echo "  cv: Computer vision (OpenCV, image processing)"
	@echo "  ocr: OCR and document processing (heavy)"
	@echo "  ml-heavy: Performance ML packages"
	@echo "  ml-training: Training and fine-tuning"
	@echo "  scientific: Scientific computing"
	@echo "  nlp: Natural language processing"
	@echo "  video: Video processing"
	@echo "  web: Web scraping"
	@echo "  formats: Additional file formats"
	@echo "  full: Everything (5GB+ download)"

# Maintenance
clean: ## Clean build artifacts and cache
	@echo "🧹 Cleaning build artifacts..."
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name ".mypy_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@rm -rf dist/ build/ .coverage htmlcov/ .tox/ .ruff_cache/
	@echo "✅ Cleanup completed!"

clean-models: ## Clean downloaded model cache
	@echo "🧹 Cleaning model cache..."
	@rm -rf ~/.cache/huggingface/transformers/
	@rm -rf ~/.cache/torch/hub/
	@echo "✅ Model cache cleaned!"

clean-outputs: ## Clean processing outputs
	@echo "🧹 Cleaning processing outputs..."
	@rm -rf outputs/ processed/ embeddings/
	@echo "✅ Output directories cleaned!"

# Virtual environment management
venv-info: ## Show virtual environment information
	@echo "🐍 Virtual Environment Information"
	@echo "=================================="
	poetry env info

venv-activate: ## Show command to activate virtual environment manually
	@echo "🐍 To manually activate the virtual environment, run:"
	@echo "source $$(poetry env info --path)/bin/activate"

venv-shell: ## Enter virtual environment shell
	@echo "🐍 Entering virtual environment shell..."
	poetry shell

venv-remove: ## Remove virtual environment
	@echo "🗑️  Removing virtual environment..."
	poetry env remove python
	@echo "✅ Virtual environment removed!"

venv-recreate: ## Recreate virtual environment
	@echo "🔄 Recreating virtual environment..."
	poetry env remove python || true
	poetry install
	@echo "✅ Virtual environment recreated!"

# Information and debugging
info: ## Show environment and installation info
	@echo "ℹ️  RAG Pipeline Environment"
	@echo "=========================="
	@echo "Python: $$(python --version 2>/dev/null || echo 'Not found')"
	@echo "Poetry: $$(poetry --version 2>/dev/null || echo 'Not found')"
	@echo "PyTorch: $$($(PYTHON) -c 'import torch; print(f\"PyTorch {torch.__version__} (CUDA: {torch.cuda.is_available()})\"' 2>/dev/null || echo 'Not installed')"
	@echo "Transformers: $$($(PYTHON) -c 'import transformers; print(transformers.__version__)' 2>/dev/null || echo 'Not installed')"
	@echo "OpenCV: $$($(PYTHON) -c 'import cv2; print(cv2.__version__)' 2>/dev/null || echo 'Not installed')"
	@echo ""
	@echo "🔧 Feature Status:"
	@echo "Basic: ✅ (always available)"
	@echo "CV: $$($(PYTHON) -c 'import cv2' 2>/dev/null && echo '✅ Available' || echo '❌ Not installed')"
	@echo "OCR: $$($(PYTHON) -c 'import paddleocr' 2>/dev/null && echo '✅ Available' || echo '❌ Not installed')"
	@echo "NLP: $$($(PYTHON) -c 'import spacy' 2>/dev/null && echo '✅ Available' || echo '❌ Not installed')"

check-deps: ## Check if optional dependencies are installed
	@echo "🔍 Checking optional dependencies..."
	@echo "Computer Vision:"
	@$(PYTHON) -c "import cv2; print('  ✅ OpenCV available')" 2>/dev/null || echo "  ❌ OpenCV not installed (make install-cv)"
	@echo "OCR:"
	@$(PYTHON) -c "import paddleocr; print('  ✅ PaddleOCR available')" 2>/dev/null || echo "  ❌ PaddleOCR not installed (make install-ocr)"
	@echo "NLP:"
	@$(PYTHON) -c "import spacy; print('  ✅ spaCy available')" 2>/dev/null || echo "  ❌ spaCy not installed (make install-nlp)"
	@echo "Video:"
	@$(PYTHON) -c "import moviepy; print('  ✅ MoviePy available')" 2>/dev/null || echo "  ❌ MoviePy not installed (make install-video)"

# Quick setup workflows
setup-basic: install-basic download-models ## Quick setup with basic features
	@echo "🎉 Basic RAG setup complete!"
	@echo "Try: make convert INPUT=your_docs/ OUTPUT=processed/"

setup-full: install-full download-models ## Complete setup with all features
	@echo "🎉 Full RAG setup complete!"
	@echo "All features available including OCR, CV, NLP, and video processing."

# Benchmarking and performance
benchmark: ## Run performance benchmarks
	@echo "⚡ Running performance benchmarks..."
	$(PYTHON) -m src.benchmarks.run_benchmarks
	@echo "✅ Benchmarks completed!"