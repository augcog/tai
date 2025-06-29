.PHONY: help install dev test lint format clean
.PHONY: add add-dev remove update show
.PHONY: generate analyze visualize

POETRY_RUN = poetry run
PYTHON = $(POETRY_RUN) python

help: ## Show this help message
	@echo "Evaluation Tools - Dataset Generation & Analysis"
	@echo "=============================================="
	@echo ""
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install: ## Install dependencies
	@echo "📦 Installing Evaluation Tools dependencies..."
	poetry install
	@echo "✅ Dependencies installed!"

dev: ## Start development environment
	@echo "🚀 Starting Jupyter notebook server..."
	$(POETRY_RUN) jupyter notebook

test: ## Run tests
	@echo "🧪 Running tests..."
	$(POETRY_RUN) pytest -v

test-no-api: ## Run tests without OpenAI API calls
	@echo "🧪 Running tests (excluding OpenAI API tests)..."
	$(POETRY_RUN) pytest -v -m "not openai"

lint: ## Run linting
	@echo "🔍 Running linting..."
	$(POETRY_RUN) ruff check .
	$(POETRY_RUN) mypy src

format: ## Format code
	@echo "🎨 Formatting code..."
	$(POETRY_RUN) black .
	$(POETRY_RUN) ruff format .

# Evaluation-specific commands
generate: ## Generate dataset (use: make generate CONFIG=config.yaml)
	@if [ -z "$(CONFIG)" ]; then echo "❌ Usage: make generate CONFIG=config.yaml"; exit 1; fi
	@echo "📊 Generating dataset with config: $(CONFIG)..."
	$(PYTHON) src/generate.py --config $(CONFIG)

analyze: ## Analyze results (use: make analyze DATA=results.json)
	@if [ -z "$(DATA)" ]; then echo "❌ Usage: make analyze DATA=results.json"; exit 1; fi
	@echo "📈 Analyzing results: $(DATA)..."
	$(PYTHON) src/analyze.py --data $(DATA)

visualize: ## Create visualizations (use: make visualize DATA=results.json)
	@if [ -z "$(DATA)" ]; then echo "❌ Usage: make visualize DATA=results.json"; exit 1; fi
	@echo "📊 Creating visualizations for: $(DATA)..."
	$(PYTHON) src/visualize.py --data $(DATA)

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
	@rm -rf .pytest_cache .mypy_cache