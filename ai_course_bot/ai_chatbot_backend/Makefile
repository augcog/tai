.PHONY: help install dev test lint format clean
.PHONY: add add-dev remove update show export
.PHONY: server db-init db-status admin test-api
.PHONY: docker-build docker-run

POETRY_RUN = poetry run
PYTHON = $(POETRY_RUN) python
PYTEST = $(POETRY_RUN) pytest

# Default target
help: ## Show this help message
	@echo "TAI Backend Service - Development Commands"
	@echo "========================================"
	@echo ""
	@echo "🚀 Quick Start:"
	@echo "  make install     # Install dependencies"
	@echo "  make dev         # Start development server"
	@echo ""
	@echo "📦 Setup Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(install|setup)"
	@echo ""
	@echo "🛠️  Development Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(dev|server|test|lint|format)"
	@echo ""
	@echo "🗄️  Database Commands:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(db-|admin)"
	@echo ""
	@echo "📦 Package Management:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(add|remove|update|show)"
	@echo ""
	@echo "🐍 Virtual Environment:"
	@awk 'BEGIN {FS = ":.*?## "} /^venv.*:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""
	@echo "💡 Examples:"
	@echo "  make add PKG=torch              # Add production dependency"
	@echo "  make add-dev PKG=pytest         # Add development dependency"
	@echo "  make remove PKG=outdated-pkg    # Remove dependency"
	@echo ""

# Installation and setup
install: ## Install all dependencies
	@echo "📦 Installing TAI Backend dependencies..."
	@echo "🔒 Creating isolated virtual environment..."
	poetry install
	@poetry env info --path | sed 's/^/🐍 Virtual environment: /'
	@echo "✅ Dependencies installed in isolated virtual environment!"

install-dev: ## Install with development dependencies
	@echo "📦 Installing with development dependencies..."
	poetry install --with dev,test
	@echo "✅ Development environment ready!"

# Development server
dev: server ## Start development server (alias for server)

server: ## Start the FastAPI development server
	@echo "🚀 Starting TAI Backend Server..."
	@echo "📍 Server will be available at: http://localhost:8000"
	@echo "📚 API Documentation: http://localhost:8000/docs"
	@echo "🔧 Admin Interface: http://localhost:8000/admin"
	@echo ""
	$(PYTHON) main.py

server-reload: ## Start server with auto-reload (for development)
	@echo "🚀 Starting TAI Backend Server with auto-reload..."
	$(POETRY_RUN) uvicorn main:app --reload --host 0.0.0.0 --port 8000

# Database operations
db-init: ## Initialize database and import files
	@echo "🗄️  Initializing database..."
	$(PYTHON) scripts/initialize_db_and_files.py
	@echo "✅ Database initialized!"

db-status: ## Check database status
	@echo "🗄️  Checking database status..."
	@curl -s http://localhost:8000/database-status 2>/dev/null || echo "❌ Server not running. Start with 'make dev'"

admin: ## Open admin interface (server must be running)
	@echo "🔧 Opening admin interface..."
	@echo "Visit: http://localhost:8000/admin"
	@command -v open >/dev/null 2>&1 && open http://localhost:8000/admin || echo "Manual: http://localhost:8000/admin"

# Testing
test: ## Run all tests
	@echo "🧪 Running tests..."
	$(PYTEST) -v

test-unit: ## Run unit tests only
	@echo "🧪 Running unit tests..."
	$(PYTEST) -v -m "unit"

test-integration: ## Run integration tests only
	@echo "🧪 Running integration tests..."
	$(PYTEST) -v -m "integration"

test-api: ## Run API tests only
	@echo "🧪 Running API tests..."
	$(PYTEST) -v -m "api"

test-cov: ## Run tests with coverage report
	@echo "🧪 Running tests with coverage..."
	$(PYTEST) --cov=app --cov-report=html --cov-report=term-missing

test-watch: ## Run tests in watch mode
	@echo "🧪 Running tests in watch mode..."
	$(PYTEST) -f

# Code quality
lint: ## Run all linting checks
	@echo "🔍 Running linting checks..."
	$(POETRY_RUN) ruff check .
	$(POETRY_RUN) mypy app
	@echo "✅ Linting completed!"

lint-fix: ## Run linting with auto-fix
	@echo "🔍 Running linting with auto-fix..."
	$(POETRY_RUN) ruff check --fix .
	$(POETRY_RUN) ruff format .

format: ## Format code with black and ruff
	@echo "🎨 Formatting code..."
	$(POETRY_RUN) black .
	$(POETRY_RUN) ruff format .
	@echo "✅ Code formatted!"

type-check: ## Run type checking
	@echo "🔍 Running type checks..."
	$(POETRY_RUN) mypy app

# Package management
add: ## Add production dependency (use: make add PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add PKG=package-name"; exit 1; fi
	@echo "📦 Adding package: $(PKG)"
	poetry add $(PKG)
	@echo "✅ Package $(PKG) added to dependencies!"

add-dev: ## Add development dependency (use: make add-dev PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add-dev PKG=package-name"; exit 1; fi
	@echo "📦 Adding development package: $(PKG)"
	poetry add --group dev $(PKG)
	@echo "✅ Development package $(PKG) added!"

remove: ## Remove dependency (use: make remove PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make remove PKG=package-name"; exit 1; fi
	@echo "📦 Removing package: $(PKG)"
	poetry remove $(PKG)
	@echo "✅ Package $(PKG) removed!"

update: ## Update all dependencies
	@echo "⬆️  Updating dependencies..."
	poetry update
	@echo "✅ Dependencies updated!"

update-pkg: ## Update specific package (use: make update-pkg PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make update-pkg PKG=package-name"; exit 1; fi
	@echo "⬆️  Updating package: $(PKG)"
	poetry update $(PKG)
	@echo "✅ Package $(PKG) updated!"

show: ## Show dependency information (use: make show PKG=package-name)
	@if [ -z "$(PKG)" ]; then \
		echo "📊 Showing all dependencies:"; \
		poetry show; \
	else \
		echo "📊 Showing package info: $(PKG)"; \
		poetry show $(PKG); \
	fi

export: ## Export requirements.txt for deployment
	@echo "📦 Exporting requirements.txt..."
	poetry export --format=requirements.txt --output=requirements.txt --without-hashes
	@echo "✅ requirements.txt generated!"

lock: ## Update poetry.lock without installing
	@echo "🔒 Updating poetry.lock..."
	poetry lock
	@echo "✅ Lock file updated!"

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

clean-logs: ## Clean log files
	@echo "🧹 Cleaning log files..."
	@rm -f logs.log *.log
	@echo "✅ Log files cleaned!"

# Docker operations (if needed)
docker-build: ## Build Docker image
	@echo "🐳 Building Docker image..."
	docker build -t tai-backend .

docker-run: ## Run Docker container
	@echo "🐳 Running Docker container..."
	docker run -p 8000:8000 --env-file .env tai-backend

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

# Health checks and info
health: ## Check service health
	@echo "🏥 Checking service health..."
	@curl -s http://localhost:8000/health 2>/dev/null || echo "❌ Service not running. Start with 'make dev'"

info: ## Show environment information
	@echo "ℹ️  Environment Information"
	@echo "=========================="
	@echo "Python: $$(python --version 2>/dev/null || echo 'Not found')"
	@echo "Poetry: $$(poetry --version 2>/dev/null || echo 'Not found')"
	@echo "FastAPI: $$($(POETRY_RUN) python -c 'import fastapi; print(fastapi.__version__)' 2>/dev/null || echo 'Not installed')"
	@echo "Working Directory: $$(pwd)"
	@echo ""
	@echo "🔧 Configuration:"
	@echo "Host: $$(grep -E '^HOST=' .env 2>/dev/null | cut -d= -f2 || echo 'localhost')"
	@echo "Port: $$(grep -E '^PORT=' .env 2>/dev/null | cut -d= -f2 || echo '8000')"
	@echo "Environment: $$(grep -E '^ENVIRONMENT=' .env 2>/dev/null | cut -d= -f2 || echo 'development')"

# Development workflow shortcuts
setup: install db-init ## Complete setup for new developers
	@echo ""
	@echo "🎉 TAI Backend setup complete!"
	@echo ""
	@echo "🚀 Next steps:"
	@echo "  make dev         # Start the development server"
	@echo "  make test        # Run the test suite"
	@echo "  make admin       # Open admin interface"

check: lint test ## Run all quality checks
	@echo "✅ All quality checks passed!"

ci: install lint test ## Run CI pipeline locally
	@echo "✅ CI pipeline completed successfully!"