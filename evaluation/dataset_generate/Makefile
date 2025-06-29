.PHONY: help install dev test lint format clean
.PHONY: add add-dev remove update show
.PHONY: generate analyze visualize

# Find the root directory (where the main pyproject.toml is)
ROOT_DIR := $(shell cd ../.. && pwd)
POETRY_RUN = cd $(ROOT_DIR) && poetry run
PYTHON = $(POETRY_RUN) python

help: ## Show this help message
	@echo "Evaluation Tools - Dataset Generation & Analysis"
	@echo "=============================================="
	@echo ""
	@echo "📦 Installs to root monorepo - Package management modifies root"
	@echo ""
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install: ## Install dependencies from root monorepo
	@echo "📦 Installing Evaluation Tools dependencies from root monorepo..."
	@echo "🔗 Using unified virtual environment at: $(ROOT_DIR)/.venv"
	cd $(ROOT_DIR) && poetry install
	@echo "✅ Dependencies installed in unified virtual environment!"

dev: ## Start development environment
	@echo "🚀 Starting Jupyter notebook server..."
	cd $(shell pwd) && $(POETRY_RUN) jupyter notebook

test: ## Run tests for evaluation tools
	@echo "🧪 Running evaluation tests..."
	$(POETRY_RUN) pytest -v tests/

test-no-api: ## Run tests without OpenAI API calls
	@echo "🧪 Running tests (excluding OpenAI API tests)..."
	$(POETRY_RUN) pytest -v -m "not openai" tests/

lint: ## Run linting on evaluation files only
	@echo "🔍 Running linting on evaluation files..."
	cd $(ROOT_DIR) && poetry run ruff check evaluation/dataset_generate/
	cd $(ROOT_DIR) && poetry run mypy evaluation/dataset_generate/src

format: ## Format evaluation code only
	@echo "🎨 Formatting evaluation code..."
	cd $(ROOT_DIR) && poetry run black evaluation/dataset_generate/
	cd $(ROOT_DIR) && poetry run ruff format evaluation/dataset_generate/

# Evaluation-specific commands
generate: ## Generate dataset (use: make generate CONFIG=config.yaml)
	@if [ -z "$(CONFIG)" ]; then echo "❌ Usage: make generate CONFIG=config.yaml"; exit 1; fi
	@echo "📊 Generating dataset with config: $(CONFIG)..."
	cd $(shell pwd) && $(PYTHON) src/generate.py --config $(CONFIG)

analyze: ## Analyze results (use: make analyze DATA=results.json)
	@if [ -z "$(DATA)" ]; then echo "❌ Usage: make analyze DATA=results.json"; exit 1; fi
	@echo "📈 Analyzing results: $(DATA)..."
	cd $(shell pwd) && $(PYTHON) src/analyze.py --data $(DATA)

visualize: ## Create visualizations (use: make visualize DATA=results.json)
	@if [ -z "$(DATA)" ]; then echo "❌ Usage: make visualize DATA=results.json"; exit 1; fi
	@echo "📊 Creating visualizations for: $(DATA)..."
	cd $(shell pwd) && $(PYTHON) src/visualize.py --data $(DATA)

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

clean: ## Clean artifacts in evaluation directory
	@echo "🧹 Cleaning evaluation artifacts..."
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@rm -rf .pytest_cache .mypy_cache
	@echo "✅ Evaluation cleanup completed!"
