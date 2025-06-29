.PHONY: help install dev test lint format clean
.PHONY: add add-dev remove update show
.PHONY: organize classify summarize

POETRY_RUN = poetry run
PYTHON = $(POETRY_RUN) python

help: ## Show this help message
	@echo "File Organizer - ML-based File Classification"
	@echo "==========================================="
	@echo ""
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install: ## Install dependencies
	@echo "📦 Installing File Organizer dependencies..."
	poetry install
	@echo "✅ Dependencies installed!"

dev: ## Development mode
	@echo "🚀 Starting File Organizer in development mode..."
	$(PYTHON) -m src.dev_server

test: ## Run tests
	@echo "🧪 Running tests..."
	$(POETRY_RUN) pytest -v

lint: ## Run linting
	@echo "🔍 Running linting..."
	$(POETRY_RUN) ruff check .
	$(POETRY_RUN) mypy src

format: ## Format code
	@echo "🎨 Formatting code..."
	$(POETRY_RUN) black .
	$(POETRY_RUN) ruff format .

# Tool-specific commands
organize: ## Organize files (use: make organize PATH=./files)
	@if [ -z "$(PATH)" ]; then echo "❌ Usage: make organize PATH=./files"; exit 1; fi
	@echo "📁 Organizing files in $(PATH)..."
	$(PYTHON) -m src.organizer --path $(PATH)

classify: ## Classify files (use: make classify PATH=./files)
	@if [ -z "$(PATH)" ]; then echo "❌ Usage: make classify PATH=./files"; exit 1; fi
	@echo "🏷️  Classifying files in $(PATH)..."
	$(PYTHON) -m src.classifier --path $(PATH)

# Package management
add: ## Add package (use: make add PKG=name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add PKG=package-name"; exit 1; fi
	poetry add $(PKG)

add-dev: ## Add dev package (use: make add-dev PKG=name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add-dev PKG=package-name"; exit 1; fi
	poetry add --group dev $(PKG)

remove: ## Remove package (use: make remove PKG=name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make remove PKG=package-name"; exit 1; fi
	poetry remove $(PKG)

update: ## Update dependencies
	poetry update

show: ## Show dependencies
	poetry show

clean: ## Clean artifacts
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true