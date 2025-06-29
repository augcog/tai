.PHONY: help install test lint format clean dev-backend dev-rag update show-deps
.PHONY: install-cv install-ocr install-ml-heavy install-full check-format type check-all

# Default target
help: ## Show this help message
	@echo "🚀 TAI Monorepo - Unified Poetry Environment"
	@echo "============================================="
	@echo ""
	@echo "📦 Quick Start:"
	@echo "  make install        # Install core dependencies"
	@echo "  make dev-backend     # Start backend service"
	@echo ""
	@echo "🔧 Installation Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(install|deps)"
	@echo ""
	@echo "🛠️  Development Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(dev|test|lint|format|clean|type|check)"
	@echo ""
	@echo "🔬 Optional Feature Groups:"
	@echo "  make install-cv      # Install computer vision packages"
	@echo "  make install-ocr     # Install OCR packages (heavy)"
	@echo "  make install-ml-heavy # Install heavy ML packages"
	@echo "  make install-full    # Install all optional packages"
	@echo ""

# Installation commands
install: ## Install core dependencies in single virtual environment
	@echo "🔥 Installing TAI unified environment..."
	@echo "📦 Installing core dependencies..."
	poetry install
	@echo "✅ Core installation complete!"
	@echo ""
	@echo "🎯 Next steps:"
	@echo "  make dev-backend     # Start the backend service"
	@echo "  make test            # Run all tests"
	@echo "  make install-cv      # Add computer vision support"
	@echo "  make install-ocr     # Add OCR support (heavy packages)"

install-cv: ## Install computer vision packages
	@echo "📦 Installing computer vision packages..."
	poetry install --with cv
	@echo "✅ Computer vision packages installed!"

install-ocr: ## Install OCR packages (heavy dependencies)
	@echo "📦 Installing OCR packages (this may take a while)..."
	poetry install --with ocr
	@echo "✅ OCR packages installed!"

install-ml-heavy: ## Install heavy ML packages
	@echo "📦 Installing heavy ML packages..."
	poetry install --with ml-heavy
	@echo "✅ Heavy ML packages installed!"

install-video: ## Install video processing packages
	@echo "📦 Installing video processing packages..."
	poetry install --with video
	@echo "✅ Video processing packages installed!"

install-web: ## Install web scraping packages
	@echo "📦 Installing web scraping packages..."
	poetry install --with web
	@echo "✅ Web scraping packages installed!"

install-formats: ## Install additional format support
	@echo "📦 Installing additional format packages..."
	poetry install --with formats
	@echo "✅ Additional format packages installed!"

install-training: ## Install ML training packages
	@echo "📦 Installing ML training packages..."
	poetry install --with ml-training
	@echo "✅ ML training packages installed!"

install-full: ## Install all optional packages (may take significant time)
	@echo "📦 Installing ALL optional packages (this will take a while)..."
	poetry install --with cv,ocr,ml-heavy,video,web,formats,ml-training
	@echo "✅ All packages installed!"

# Development commands
dev-backend: ## Start the backend development server
	@echo "🚀 Starting TAI Backend Service..."
	@cd ai_course_bot/ai_chatbot_backend && poetry run python main.py

dev-rag: ## Start RAG development tools
	@echo "🚀 Starting RAG Pipeline Tools..."
	@cd rag && poetry run python -m file_conversion_router.api

dev: dev-backend ## Alias for dev-backend

# Testing commands
test: ## Run all tests in unified environment
	@echo "🧪 Running all tests..."
	poetry run pytest
	@echo "✅ All tests completed!"

test-backend: ## Run backend tests only
	@echo "🧪 Running backend tests..."
	poetry run pytest ai_course_bot/ai_chatbot_backend/tests/
	@echo "✅ Backend tests completed!"

test-rag: ## Run RAG pipeline tests only
	@echo "🧪 Running RAG tests..."
	poetry run pytest rag/tests/
	@echo "✅ RAG tests completed!"

test-evaluation: ## Run evaluation tests only
	@echo "🧪 Running evaluation tests..."
	poetry run pytest evaluation/dataset_generate/tests/
	@echo "✅ Evaluation tests completed!"

test-organizer: ## Run file organizer tests only
	@echo "🧪 Running file organizer tests..."
	poetry run pytest rag/file_organizer/tests/
	@echo "✅ File organizer tests completed!"

# Code quality commands
lint: ## Run linting across all code
	@echo "🔍 Running linting..."
	poetry run ruff check .
	@echo "✅ Linting completed!"

format: ## Format code across all projects
	@echo "🎨 Formatting code..."
	poetry run black .
	poetry run ruff format .
	@echo "✅ Code formatting completed!"

check-format: ## Check if code is formatted correctly
	@echo "🔍 Checking code format..."
	poetry run black --check .
	poetry run ruff format --check .
	@echo "✅ Format check completed!"

type: ## Run type checking
	@echo "🔍 Running type checks..."
	poetry run mypy ai_course_bot/ai_chatbot_backend/app --ignore-missing-imports || true
	poetry run mypy rag/file_conversion_router --ignore-missing-imports || true
	poetry run mypy evaluation/dataset_generate --ignore-missing-imports || true
	poetry run mypy rag/file_organizer/src --ignore-missing-imports || true
	@echo "✅ Type checking completed!"

check-all: check-format lint type ## Run all code quality checks
	@echo "✅ All code quality checks completed!"

# Maintenance commands
clean: ## Clean build artifacts and caches
	@echo "🧹 Cleaning project..."
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name ".mypy_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@echo "✅ Cleanup completed!"

update: ## Update all dependencies
	@echo "⬆️  Updating dependencies..."
	poetry update
	@echo "✅ Dependencies updated!"

show-deps: ## Show dependency tree
	@echo "📊 Dependency tree:"
	poetry show --tree

# Package management commands (examples)
add: ## Add a package (usage: make add PKG=package-name)
	@if [ -z "$(PKG)" ]; then \
		echo "❌ Usage: make add PKG=package-name"; \
		echo "   Example: make add PKG=torch"; \
		exit 1; \
	fi
	@echo "📦 Adding package: $(PKG)"
	poetry add $(PKG)

add-dev: ## Add a development package (usage: make add-dev PKG=package-name)
	@if [ -z "$(PKG)" ]; then \
		echo "❌ Usage: make add-dev PKG=package-name"; \
		echo "   Example: make add-dev PKG=pytest"; \
		exit 1; \
	fi
	@echo "📦 Adding dev package: $(PKG)"
	poetry add --group dev $(PKG)

remove: ## Remove a package (usage: make remove PKG=package-name)
	@if [ -z "$(PKG)" ]; then \
		echo "❌ Usage: make remove PKG=package-name"; \
		echo "   Example: make remove PKG=torch"; \
		exit 1; \
	fi
	@echo "📦 Removing package: $(PKG)"
	poetry remove $(PKG)

show: ## Show package info (usage: make show PKG=package-name)
	@if [ -z "$(PKG)" ]; then \
		echo "❌ Usage: make show PKG=package-name"; \
		echo "   Example: make show PKG=torch"; \
		exit 1; \
	fi
	@echo "📊 Package info: $(PKG)"
	poetry show $(PKG)

# Utility commands
check-poetry: ## Check if Poetry is installed
	@which poetry > /dev/null || (echo "❌ Poetry not found. Install with: curl -sSL https://install.python-poetry.org | python3 -" && exit 1)
	@echo "✅ Poetry is installed: $$(poetry --version)"

setup: check-poetry install ## Complete setup from scratch
	@echo ""
	@echo "🎉 TAI unified environment setup complete!"
	@echo ""
	@echo "🚀 Try these commands:"
	@echo "  make dev-backend     # Start the backend service"
	@echo "  make test            # Run all tests"
	@echo "  make lint            # Check code quality"
	@echo "  make install-cv      # Add computer vision support"

# Status check
status: ## Show status of the unified environment
	@echo "📋 TAI Monorepo Status"
	@echo "====================="
	@echo ""
	@echo "🔧 Environment:"
	@poetry --version 2>/dev/null || echo "❌ Poetry not found"
	@python --version 2>/dev/null || echo "❌ Python not found"
	@echo ""
	@echo "📦 Virtual Environment:"
	@poetry env info --path 2>/dev/null || echo "❌ No virtual environment found"
	@echo ""
	@echo "🧪 Dependencies Status:"
	@echo "  Core:        $$(poetry show | wc -l | xargs) packages installed"
	@echo "  Dev tools:   $$(poetry show --only dev | wc -l | xargs) packages"
	@echo ""
	@echo "🔍 Optional Groups Available:"
	@echo "  Computer Vision  # make install-cv"
	@echo "  OCR Processing   # make install-ocr"
	@echo "  Heavy ML         # make install-ml-heavy"
	@echo "  Video Processing # make install-video"
	@echo "  Web Scraping     # make install-web"
	@echo "  Additional Formats # make install-formats"
	@echo "  ML Training      # make install-training"

# Entry point commands using unified environment
backend: ## Run backend server using unified environment
	@echo "🚀 Running backend server..."
	poetry run tai-backend

rag-convert: ## Run RAG file conversion (usage: make rag-convert ARGS="--help")
	@echo "🔄 Running RAG conversion..."
	poetry run rag-convert $(ARGS)

rag-embed: ## Run RAG embedding generation (usage: make rag-embed ARGS="--help")
	@echo "🧠 Running RAG embedding generation..."
	poetry run rag-embed $(ARGS)

generate-dataset: ## Run dataset generation (usage: make generate-dataset ARGS="--help")
	@echo "📊 Running dataset generation..."
	poetry run generate-dataset $(ARGS)

file-organizer: ## Run file organizer (usage: make file-organizer ARGS="--help")
	@echo "📁 Running file organizer..."
	poetry run file-organizer $(ARGS)

# Migration helpers (for transitioning from old setup)
migrate-old-envs: ## Remove old individual virtual environments and lock files
	@echo "🔄 Cleaning up old individual environments..."
	@echo "   Removing old poetry.lock files..."
	@rm -f ai_course_bot/ai_chatbot_backend/poetry.lock 2>/dev/null || true
	@rm -f rag/poetry.lock 2>/dev/null || true
	@rm -f evaluation/dataset_generate/poetry.lock 2>/dev/null || true
	@rm -f rag/file_organizer/poetry.lock 2>/dev/null || true
	@echo "   Removing old pyproject.toml files (keeping backups)..."
	@cp ai_course_bot/ai_chatbot_backend/pyproject.toml ai_course_bot/ai_chatbot_backend/pyproject.toml.backup 2>/dev/null || true
	@cp rag/pyproject.toml rag/pyproject.toml.backup 2>/dev/null || true
	@cp evaluation/dataset_generate/pyproject.toml evaluation/dataset_generate/pyproject.toml.backup 2>/dev/null || true
	@cp rag/file_organizer/pyproject.toml rag/file_organizer/pyproject.toml.backup 2>/dev/null || true
	@echo "✅ Old environments cleaned up (backups created)"
	@echo ""
	@echo "🎯 Now run:"
	@echo "  make install         # Install unified environment"