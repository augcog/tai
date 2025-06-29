.PHONY: help install dev test lint format clean
.PHONY: add add-dev remove update show
.PHONY: organize classify summarize

# Find the root directory (where the main pyproject.toml is)
ROOT_DIR := $(shell cd ../.. && pwd)
POETRY_RUN = cd $(ROOT_DIR) && poetry run
PYTHON = $(POETRY_RUN) python

help: ## Show this help message
	@echo "File Organizer - ML-based File Classification"
	@echo "==========================================="
	@echo ""
	@echo "📦 Installs to root monorepo - Package management modifies root"
	@echo ""
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install: ## Install dependencies from root monorepo
	@echo "📦 Installing File Organizer dependencies from root monorepo..."
	@echo "🔗 Using unified virtual environment at: $(ROOT_DIR)/.venv"
	cd $(ROOT_DIR) && poetry install
	@echo "✅ Dependencies installed in unified virtual environment!"

dev: ## Development mode
	@echo "🚀 Starting File Organizer in development mode..."
	cd $(shell pwd) && $(PYTHON) -m src.dev_server

test: ## Run tests for file organizer
	@echo "🧪 Running file organizer tests..."
	$(POETRY_RUN) pytest -v tests/

lint: ## Run linting on file organizer files only
	@echo "🔍 Running linting on file organizer files..."
	cd $(ROOT_DIR) && poetry run ruff check rag/file_organizer/
	cd $(ROOT_DIR) && poetry run mypy rag/file_organizer/src

format: ## Format file organizer code only
	@echo "🎨 Formatting file organizer code..."
	cd $(ROOT_DIR) && poetry run black rag/file_organizer/
	cd $(ROOT_DIR) && poetry run ruff format rag/file_organizer/

# Tool-specific commands
organize: ## Organize files (use: make organize PATH=./files)
	@if [ -z "$(PATH)" ]; then echo "❌ Usage: make organize PATH=./files"; exit 1; fi
	@echo "📁 Organizing files in $(PATH)..."
	cd $(shell pwd) && $(PYTHON) -m src.organizer --path $(PATH)

classify: ## Classify files (use: make classify PATH=./files)
	@if [ -z "$(PATH)" ]; then echo "❌ Usage: make classify PATH=./files"; exit 1; fi
	@echo "🏷️  Classifying files in $(PATH)..."
	cd $(shell pwd) && $(PYTHON) -m src.classifier --path $(PATH)

# Package management - Always modifies root pyproject.toml
add: ## Add package to root monorepo (use: make add PKG=name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add PKG=package-name"; exit 1; fi
	@echo "📦 Adding package $(PKG) to root monorepo..."
	cd $(ROOT_DIR) && poetry add $(PKG)
	@echo "✅ Package $(PKG) added to root!"

add-dev: ## Add dev package to root monorepo (use: make add-dev PKG=name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add-dev PKG=package-name"; exit 1; fi
	@echo "📦 Adding dev package $(PKG) to root monorepo..."
	cd $(ROOT_DIR) && poetry add --group dev $(PKG)
	@echo "✅ Dev package $(PKG) added to root!"

remove: ## Remove package from root monorepo (use: make remove PKG=name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make remove PKG=package-name"; exit 1; fi
	@echo "📦 Removing package $(PKG) from root monorepo..."
	cd $(ROOT_DIR) && poetry remove $(PKG)
	@echo "✅ Package $(PKG) removed from root!"

update: ## Update dependencies in root monorepo
	@echo "⬆️  Updating dependencies in root monorepo..."
	cd $(ROOT_DIR) && poetry update
	@echo "✅ Root dependencies updated!"

show: ## Show dependencies from root monorepo
	@echo "📊 Showing dependencies from root monorepo:"
	cd $(ROOT_DIR) && poetry show

clean: ## Clean artifacts in file organizer directory
	@echo "🧹 Cleaning file organizer artifacts..."
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@echo "✅ File organizer cleanup completed!"
