.PHONY: help install install-basic install-cv install-ocr install-full
.PHONY: dev test lint format clean
.PHONY: add add-dev remove update show
.PHONY: convert embed process
.PHONY: test-cv test-ocr test-ml
.PHONY: setup-models download-models

# Find the root directory (where the main pyproject.toml is)
ROOT_DIR := $(shell cd .. && pwd)
POETRY_RUN = cd $(ROOT_DIR) && poetry run
PYTHON = $(POETRY_RUN) python
PYTEST = $(POETRY_RUN) pytest

# Default target
help: ## Show this help message
	@echo "RAG Pipeline - Document Processing & Embedding Generation"
	@echo "========================================================"
	@echo ""
	@echo "🚀 Quick Start:"
	@echo "  make install-basic    # Install core dependencies from root monorepo"
	@echo "  make install-full     # Install everything from root monorepo (large download!)"
	@echo ""
	@echo "📦 Installation Options (installs to root monorepo):"
	@awk 'BEGIN {FS = ":.*?## "} /^install.*:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""
	@echo "🛠️  Development Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(dev|test|lint|format|clean)"
	@echo ""
	@echo "🔄 Processing Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(convert|embed|process|models)"
	@echo ""
	@echo "📦 Package Management (modifies root monorepo):"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(add|remove|update|show)"
	@echo ""
	@echo "💡 Examples:"
	@echo "  make convert INPUT=docs/ OUTPUT=processed/"
	@echo "  make add PKG=torch GROUP=ml-heavy      # Add to root monorepo"
	@echo "  make install-cv                       # Add CV support to root"
	@echo ""

# Installation commands - all install to root monorepo
install: install-basic ## Install basic dependencies from root (alias for install-basic)

install-basic: ## Install core dependencies from root monorepo (fastest)
	@echo "📦 Installing basic RAG dependencies from root monorepo..."
	@echo "🔗 Using unified virtual environment at: $(ROOT_DIR)/.venv"
	cd $(ROOT_DIR) && poetry install
	@cd $(ROOT_DIR) && poetry env info --path | sed 's/^/🐍 Virtual environment: /'
	@echo "✅ Basic dependencies installed in unified virtual environment!"
	@echo "💡 For more features: make install-cv, make install-ocr, make install-full"

install-cv: ## Install with computer vision support in root monorepo
	@echo "📦 Installing with computer vision support in root monorepo..."
	cd $(ROOT_DIR) && poetry install --with cv
	@echo "✅ Computer vision dependencies installed in root!"

install-ocr: ## Install with OCR support in root monorepo (large download)
	@echo "📦 Installing with OCR support in root monorepo (this may take a while)..."
	cd $(ROOT_DIR) && poetry install --with ocr
	@echo "✅ OCR dependencies installed in root!"

install-ml-heavy: ## Install heavy ML packages in root monorepo for performance
	@echo "📦 Installing heavy ML packages in root monorepo..."
	cd $(ROOT_DIR) && poetry install --with ml-heavy
	@echo "✅ Heavy ML packages installed in root!"

install-nlp: ## Install NLP processing packages in root monorepo
	@echo "📦 Installing NLP packages in root monorepo..."
	cd $(ROOT_DIR) && poetry install --with nlp
	@echo "✅ NLP packages installed in root!"

install-video: ## Install video processing support in root monorepo
	@echo "📦 Installing video processing support in root monorepo..."
	cd $(ROOT_DIR) && poetry install --with video
	@echo "✅ Video processing packages installed in root!"

install-web: ## Install web scraping support in root monorepo
	@echo "📦 Installing web scraping support in root monorepo..."
	cd $(ROOT_DIR) && poetry install --with web
	@echo "✅ Web scraping packages installed in root!"

install-full: ## Install all features in root monorepo (LARGE download - 5GB+)
	@echo "🚨 WARNING: This will download 5GB+ of dependencies!"
	@echo "📦 Installing ALL RAG features in root monorepo..."
	cd $(ROOT_DIR) && poetry install --with cv,ocr,ml-heavy,ml-training,video,web,formats
	@echo "✅ Full installation complete in root! All features available."

install-dev: ## Install with development dependencies in root monorepo
	@echo "📦 Installing with development dependencies in root monorepo..."
	cd $(ROOT_DIR) && poetry install --with dev
	@echo "✅ Development environment ready in root!"

# Development
dev: ## Start development server
	@echo "🚀 Starting RAG development server..."
	cd $(shell pwd) && $(PYTHON) -m file_conversion_router.api

dev-api: ## Start FastAPI development server
	@echo "🚀 Starting RAG API server..."
	cd $(shell pwd) && $(POETRY_RUN) uvicorn file_conversion_router.api:app --reload --port 8001

# Testing with feature-specific tests - tests only RAG files
test: ## Run basic tests for RAG components
	@echo "🧪 Running basic RAG tests..."
	$(PYTEST) -v -m "not slow and not cv and not ocr and not gpu" tests/

test-all: ## Run all RAG tests (requires full installation)
	@echo "🧪 Running all RAG tests..."
	$(PYTEST) -v tests/

test-unit: ## Run unit tests only for RAG
	@echo "🧪 Running RAG unit tests..."
	$(PYTEST) -v -m "unit" tests/

test-integration: ## Run integration tests for RAG
	@echo "🧪 Running RAG integration tests..."
	$(PYTEST) -v -m "integration" tests/

test-cv: ## Run computer vision tests (requires CV packages)
	@echo "🧪 Running computer vision tests..."
	$(PYTEST) -v -m "cv" tests/

test-ocr: ## Run OCR tests (requires OCR packages)
	@echo "🧪 Running OCR tests..."
	$(PYTEST) -v -m "ocr" tests/

test-ml: ## Run ML tests (requires ML packages)
	@echo "🧪 Running ML tests..."
	$(PYTEST) -v -m "not slow" --timeout=300 tests/

test-slow: ## Run slow/heavy tests
	@echo "🧪 Running slow tests (this may take a while)..."
	$(PYTEST) -v -m "slow" --timeout=1800 tests/

# Code quality - only checks RAG files
lint: ## Run linting checks on RAG files only
	@echo "🔍 Running linting checks on RAG files..."
	cd $(ROOT_DIR) && poetry run ruff check rag/
	cd $(ROOT_DIR) && poetry run mypy rag/file_conversion_router --ignore-missing-imports
	@echo "✅ RAG linting completed!"

format: ## Format RAG code only
	@echo "🎨 Formatting RAG code..."
	cd $(ROOT_DIR) && poetry run black rag/
	cd $(ROOT_DIR) && poetry run ruff format rag/
	@echo "✅ RAG code formatted!"

# RAG Processing commands
convert: ## Convert files (use: make convert INPUT=path/to/input OUTPUT=path/to/output)
	@if [ -z "$(INPUT)" ] || [ -z "$(OUTPUT)" ]; then \
		echo "❌ Usage: make convert INPUT=input_path OUTPUT=output_path"; \
		echo "Example: make convert INPUT=docs/ OUTPUT=processed/"; \
		exit 1; \
	fi
	@echo "🔄 Converting files from $(INPUT) to $(OUTPUT)..."
	cd $(shell pwd) && $(PYTHON) -m file_conversion_router.api --input $(INPUT) --output $(OUTPUT)
	@echo "✅ File conversion completed!"

embed: ## Create embeddings (use: make embed INPUT=processed/ OUTPUT=embeddings/)
	@if [ -z "$(INPUT)" ] || [ -z "$(OUTPUT)" ]; then \
		echo "❌ Usage: make embed INPUT=processed_path OUTPUT=embeddings_path"; \
		echo "Example: make embed INPUT=processed/ OUTPUT=embeddings/"; \
		exit 1; \
	fi
	@echo "🧠 Creating embeddings from $(INPUT) to $(OUTPUT)..."
	cd $(shell pwd) && $(PYTHON) -m file_conversion_router.embedding.table_create --input $(INPUT) --output $(OUTPUT)
	@echo "✅ Embedding creation completed!"

process: ## Full pipeline (use: make process INPUT=docs/ OUTPUT=final/)
	@if [ -z "$(INPUT)" ] || [ -z "$(OUTPUT)" ]; then \
		echo "❌ Usage: make process INPUT=input_path OUTPUT=output_path"; \
		echo "Example: make process INPUT=docs/ OUTPUT=final/"; \
		exit 1; \
	fi
	@echo "🔄 Running full RAG pipeline..."
	@echo "Step 1: Converting files..."
	cd $(shell pwd) && $(PYTHON) -m file_conversion_router.api --input $(INPUT) --output $(OUTPUT)/processed
	@echo "Step 2: Creating embeddings..."
	cd $(shell pwd) && $(PYTHON) -m file_conversion_router.embedding.table_create --input $(OUTPUT)/processed --output $(OUTPUT)/embeddings
	@echo "✅ Full pipeline completed!"

# Model management
download-models: ## Download required models
	@echo "📥 Downloading required models..."
	cd $(shell pwd) && $(PYTHON) -c "from file_conversion_router.services import download_base_models; download_base_models()"
	@echo "✅ Models downloaded!"

setup-models: ## Setup and verify models
	@echo "🔧 Setting up models..."
	cd $(shell pwd) && $(PYTHON) -c "from file_conversion_router.services import setup_models; setup_models()"
	@echo "✅ Models setup completed!"

# Package management - Always modifies root pyproject.toml
add: ## Add production dependency to root monorepo (use: make add PKG=package-name [GROUP=group-name])
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add PKG=package-name [GROUP=group-name]"; exit 1; fi
	@if [ -n "$(GROUP)" ]; then \
		echo "📦 Adding $(PKG) to group $(GROUP) in root monorepo..."; \
		cd $(ROOT_DIR) && poetry add --group $(GROUP) $(PKG); \
	else \
		echo "📦 Adding package $(PKG) to root monorepo..."; \
		cd $(ROOT_DIR) && poetry add $(PKG); \
	fi
	@echo "✅ Package $(PKG) added to root!"

add-dev: ## Add development dependency to root monorepo (use: make add-dev PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add-dev PKG=package-name"; exit 1; fi
	@echo "📦 Adding development package $(PKG) to root monorepo..."
	cd $(ROOT_DIR) && poetry add --group dev $(PKG)
	@echo "✅ Development package $(PKG) added to root!"

add-optional: ## Add to optional group in root monorepo (use: make add-optional PKG=package GROUP=cv)
	@if [ -z "$(PKG)" ] || [ -z "$(GROUP)" ]; then \
		echo "❌ Usage: make add-optional PKG=package-name GROUP=group-name"; \
		echo "Available groups: cv, ocr, ml-heavy, nlp, video, web, formats"; \
		exit 1; \
	fi
	@echo "📦 Adding $(PKG) to optional group $(GROUP) in root monorepo..."
	cd $(ROOT_DIR) && poetry add --group $(GROUP) $(PKG)
	@echo "✅ Package $(PKG) added to $(GROUP) group in root!"

remove: ## Remove dependency from root monorepo (use: make remove PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make remove PKG=package-name"; exit 1; fi
	@echo "📦 Removing package $(PKG) from root monorepo..."
	cd $(ROOT_DIR) && poetry remove $(PKG)
	@echo "✅ Package $(PKG) removed from root!"

update: ## Update all dependencies in root monorepo
	@echo "⬆️  Updating dependencies in root monorepo..."
	cd $(ROOT_DIR) && poetry update
	@echo "✅ Root dependencies updated!"

show: ## Show dependency information from root monorepo
	@if [ -z "$(PKG)" ]; then \
		echo "📊 Showing all dependencies from root:"; \
		cd $(ROOT_DIR) && poetry show; \
	else \
		echo "📊 Showing package info: $(PKG)"; \
		cd $(ROOT_DIR) && poetry show $(PKG); \
	fi

show-groups: ## Show available dependency groups
	@echo "📊 Available dependency groups:"
	@echo "  Core: Always installed"
	@echo "  cv: Computer vision (OpenCV, image processing)"
	@echo "  ocr: OCR and document processing (heavy)"
	@echo "  ml-heavy: Performance ML packages"
	@echo "  ml-training: Training and fine-tuning"
	@echo "  video: Video processing"
	@echo "  web: Web scraping"
	@echo "  formats: Additional file formats"
	@echo "  full: Everything (5GB+ download)"

# Maintenance
clean: ## Clean build artifacts and cache in RAG directory
	@echo "🧹 Cleaning RAG build artifacts..."
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name ".mypy_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@rm -rf dist/ build/ .coverage htmlcov/ .tox/ .ruff_cache/
	@echo "✅ RAG cleanup completed!"

clean-models: ## Clean downloaded model cache
	@echo "🧹 Cleaning model cache..."
	@rm -rf ~/.cache/huggingface/transformers/
	@rm -rf ~/.cache/torch/hub/
	@echo "✅ Model cache cleaned!"

clean-outputs: ## Clean processing outputs in RAG directory
	@echo "🧹 Cleaning RAG processing outputs..."
	@rm -rf outputs/ processed/ embeddings/
	@echo "✅ RAG output directories cleaned!"

# Virtual environment management - references root environment
venv-info: ## Show root virtual environment information
	@echo "🐍 Root Virtual Environment Information"
	@echo "======================================="
	cd $(ROOT_DIR) && poetry env info

venv-activate: ## Show command to activate root virtual environment manually
	@echo "🐍 To manually activate the root virtual environment, run:"
	@echo "cd $(ROOT_DIR) && source $$(cd $(ROOT_DIR) && poetry env info --path)/bin/activate"

venv-shell: ## Enter root virtual environment shell
	@echo "🐍 Entering root virtual environment shell..."
	cd $(ROOT_DIR) && poetry shell

venv-remove: ## Remove root virtual environment
	@echo "🗑️  Removing root virtual environment..."
	cd $(ROOT_DIR) && poetry env remove python
	@echo "✅ Root virtual environment removed!"

venv-recreate: ## Recreate root virtual environment
	@echo "🔄 Recreating root virtual environment..."
	cd $(ROOT_DIR) && poetry env remove python || true
	cd $(ROOT_DIR) && poetry install
	@echo "✅ Root virtual environment recreated!"

# Information and debugging
info: ## Show environment and installation info
	@echo "ℹ️  RAG Pipeline Environment"
	@echo "=========================="
	@echo "Python: $$(python --version 2>/dev/null || echo 'Not found')"
	@echo "Poetry: $$(poetry --version 2>/dev/null || echo 'Not found')"
	@echo "PyTorch: $$($(POETRY_RUN) python -c 'import torch; print(f\"PyTorch {torch.__version__} (CUDA: {torch.cuda.is_available()})\"' 2>/dev/null || echo 'Not installed')"
	@echo "Transformers: $$($(POETRY_RUN) python -c 'import transformers; print(transformers.__version__)' 2>/dev/null || echo 'Not installed')"
	@echo "OpenCV: $$($(POETRY_RUN) python -c 'import cv2; print(cv2.__version__)' 2>/dev/null || echo 'Not installed')"
	@echo "Working Directory: $$(pwd)"
	@echo "Root Directory: $(ROOT_DIR)"
	@echo ""
	@echo "🔧 Feature Status:"
	@echo "Basic: ✅ (always available)"
	@echo "CV: $$($(POETRY_RUN) python -c 'import cv2' 2>/dev/null && echo '✅ Available' || echo '❌ Not installed')"
	@echo "OCR: $$($(POETRY_RUN) python -c 'import paddleocr' 2>/dev/null && echo '✅ Available' || echo '❌ Not installed')"
	@echo "NLP: $$($(POETRY_RUN) python -c 'import spacy' 2>/dev/null && echo '✅ Available' || echo '❌ Not installed')"

check-deps: ## Check if optional dependencies are installed
	@echo "🔍 Checking optional dependencies..."
	@echo "Computer Vision:"
	@$(POETRY_RUN) python -c "import cv2; print('  ✅ OpenCV available')" 2>/dev/null || echo "  ❌ OpenCV not installed (make install-cv)"
	@echo "OCR:"
	@$(POETRY_RUN) python -c "import paddleocr; print('  ✅ PaddleOCR available')" 2>/dev/null || echo "  ❌ PaddleOCR not installed (make install-ocr)"
	@echo "NLP:"
	@$(POETRY_RUN) python -c "import spacy; print('  ✅ spaCy available')" 2>/dev/null || echo "  ❌ spaCy not installed (make install-nlp)"
	@echo "Video:"
	@$(POETRY_RUN) python -c "import moviepy; print('  ✅ MoviePy available')" 2>/dev/null || echo "  ❌ MoviePy not installed (make install-video)"

# Quick setup workflows
setup-basic: install-basic download-models ## Quick setup with basic features in root monorepo
	@echo "🎉 Basic RAG setup complete in root monorepo!"
	@echo "Try: make convert INPUT=your_docs/ OUTPUT=processed/"

setup-full: install-full download-models ## Complete setup with all features in root monorepo
	@echo "🎉 Full RAG setup complete in root monorepo!"
	@echo "All features available including OCR, CV, NLP, and video processing."

# Benchmarking and performance
benchmark: ## Run performance benchmarks
	@echo "⚡ Running performance benchmarks..."
	cd $(shell pwd) && $(PYTHON) -m file_conversion_router.benchmarks.run_benchmarks
	@echo "✅ Benchmarks completed!"
