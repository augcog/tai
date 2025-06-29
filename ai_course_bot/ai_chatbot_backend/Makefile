.PHONY: help install dev test lint format clean
.PHONY: add add-dev remove update show export
.PHONY: server db-init db-status admin test-api
.PHONY: docker-build docker-run

# Find the root directory (where the main pyproject.toml is)
ROOT_DIR := $(shell cd ../.. && pwd)
POETRY_RUN = cd $(ROOT_DIR) && poetry run
PYTHON = $(POETRY_RUN) python
PYTEST = $(POETRY_RUN) pytest

# Default target
help: ## Show this help message
	@echo "TAI Backend Service - Development Commands"
	@echo "========================================"
	@echo ""
	@echo "🚀 Quick Start:"
	@echo "  make install     # Install dependencies (root monorepo)"
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
	@echo "📦 Package Management (adds to root monorepo):"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | grep -E "(add|remove|update|show)"
	@echo ""
	@echo "🐍 Virtual Environment:"
	@awk 'BEGIN {FS = ":.*?## "} /^venv.*:.*?## / {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""
	@echo "💡 Examples:"
	@echo "  make add PKG=torch              # Add production dependency to root"
	@echo "  make add-dev PKG=pytest         # Add development dependency to root"
	@echo "  make remove PKG=outdated-pkg    # Remove dependency from root"
	@echo ""

# Installation and setup - Always installs from root
install: ## Install all dependencies from root monorepo
	@echo "📦 Installing TAI monorepo dependencies from root..."
	@echo "🔗 Using unified virtual environment at: $(ROOT_DIR)/.venv"
	cd $(ROOT_DIR) && poetry install
	@cd $(ROOT_DIR) && poetry env info --path | sed 's/^/🐍 Virtual environment: /'
	@echo "✅ Dependencies installed in unified virtual environment!"

install-dev: ## Install with development dependencies from root
	@echo "📦 Installing with development dependencies from root..."
	cd $(ROOT_DIR) && poetry install --with dev,test
	@echo "✅ Development environment ready!"

# Development server
dev: server ## Start development server (alias for server)

server: ## Start the FastAPI development server
	@echo "🚀 Starting TAI Backend Server..."
	@echo "📍 Server will be available at: http://localhost:8000"
	@echo "📚 API Documentation: http://localhost:8000/docs"
	@echo "🔧 Admin Interface: http://localhost:8000/admin"
	@echo ""
	cd $(shell pwd) && $(PYTHON) main.py

server-reload: ## Start server with auto-reload (for development)
	@echo "🚀 Starting TAI Backend Server with auto-reload..."
	cd $(shell pwd) && $(POETRY_RUN) uvicorn main:app --reload --host 0.0.0.0 --port 8000

# Database operations
db-init: ## Initialize database and import files
	@echo "🗄️  Initializing database..."
	cd $(shell pwd) && $(PYTHON) scripts/initialize_db_and_files.py
	@echo "✅ Database initialized!"

db-status: ## Check database status
	@echo "🗄️  Checking database status..."
	@curl -s http://localhost:8000/database-status 2>/dev/null || echo "❌ Server not running. Start with 'make dev'"

admin: ## Open admin interface (server must be running)
	@echo "🔧 Opening admin interface..."
	@echo "Visit: http://localhost:8000/admin"
	@command -v open >/dev/null 2>&1 && open http://localhost:8000/admin || echo "Manual: http://localhost:8000/admin"

# Testing - runs from unified environment but tests local files
test: ## Run all tests for this component
	@echo "🧪 Running backend tests..."
	$(PYTEST) -v tests/

test-unit: ## Run unit tests only
	@echo "🧪 Running unit tests..."
	$(PYTEST) -v -m "unit" tests/

test-integration: ## Run integration tests only
	@echo "🧪 Running integration tests..."
	$(PYTEST) -v -m "integration" tests/

test-api: ## Run API tests only
	@echo "🧪 Running API tests..."
	$(PYTEST) -v -m "api" tests/

test-cov: ## Run tests with coverage report
	@echo "🧪 Running tests with coverage..."
	$(PYTEST) --cov=app --cov-report=html --cov-report=term-missing tests/

test-watch: ## Run tests in watch mode
	@echo "🧪 Running tests in watch mode..."
	$(PYTEST) -f tests/

# Code quality - only checks files in current directory
lint: ## Run linting checks on backend files only
	@echo "🔍 Running linting checks on backend files..."
	cd $(ROOT_DIR) && poetry run ruff check ai_course_bot/ai_chatbot_backend/
	cd $(ROOT_DIR) && poetry run mypy ai_course_bot/ai_chatbot_backend/app
	@echo "✅ Backend linting completed!"

lint-fix: ## Run linting with auto-fix on backend files
	@echo "🔍 Running linting with auto-fix on backend files..."
	cd $(ROOT_DIR) && poetry run ruff check --fix ai_course_bot/ai_chatbot_backend/
	cd $(ROOT_DIR) && poetry run ruff format ai_course_bot/ai_chatbot_backend/

format: ## Format backend code only
	@echo "🎨 Formatting backend code..."
	cd $(ROOT_DIR) && poetry run black ai_course_bot/ai_chatbot_backend/
	cd $(ROOT_DIR) && poetry run ruff format ai_course_bot/ai_chatbot_backend/
	@echo "✅ Backend code formatted!"

type-check: ## Run type checking on backend files
	@echo "🔍 Running type checks on backend files..."
	cd $(ROOT_DIR) && poetry run mypy ai_course_bot/ai_chatbot_backend/app

# Package management - Always modifies root pyproject.toml
add: ## Add production dependency to root monorepo (use: make add PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add PKG=package-name"; exit 1; fi
	@echo "📦 Adding package $(PKG) to root monorepo..."
	cd $(ROOT_DIR) && poetry add $(PKG)
	@echo "✅ Package $(PKG) added to root dependencies!"

add-dev: ## Add development dependency to root monorepo (use: make add-dev PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make add-dev PKG=package-name"; exit 1; fi
	@echo "📦 Adding development package $(PKG) to root monorepo..."
	cd $(ROOT_DIR) && poetry add --group dev $(PKG)
	@echo "✅ Development package $(PKG) added to root!"

remove: ## Remove dependency from root monorepo (use: make remove PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make remove PKG=package-name"; exit 1; fi
	@echo "📦 Removing package $(PKG) from root monorepo..."
	cd $(ROOT_DIR) && poetry remove $(PKG)
	@echo "✅ Package $(PKG) removed from root!"

update: ## Update all dependencies in root monorepo
	@echo "⬆️  Updating dependencies in root monorepo..."
	cd $(ROOT_DIR) && poetry update
	@echo "✅ Root dependencies updated!"

update-pkg: ## Update specific package in root monorepo (use: make update-pkg PKG=package-name)
	@if [ -z "$(PKG)" ]; then echo "❌ Usage: make update-pkg PKG=package-name"; exit 1; fi
	@echo "⬆️  Updating package $(PKG) in root monorepo..."
	cd $(ROOT_DIR) && poetry update $(PKG)
	@echo "✅ Package $(PKG) updated in root!"

show: ## Show dependency information from root monorepo (use: make show PKG=package-name)
	@if [ -z "$(PKG)" ]; then \
		echo "📊 Showing all dependencies from root:"; \
		cd $(ROOT_DIR) && poetry show; \
	else \
		echo "📊 Showing package info: $(PKG)"; \
		cd $(ROOT_DIR) && poetry show $(PKG); \
	fi

export: ## Export requirements.txt for deployment
	@echo "📦 Exporting requirements.txt from root..."
	cd $(ROOT_DIR) && poetry export --format=requirements.txt --output=ai_course_bot/ai_chatbot_backend/requirements.txt --without-hashes
	@echo "✅ requirements.txt generated!"

lock: ## Update poetry.lock in root monorepo
	@echo "🔒 Updating poetry.lock in root monorepo..."
	cd $(ROOT_DIR) && poetry lock
	@echo "✅ Root lock file updated!"

# Maintenance
clean: ## Clean build artifacts and cache in backend directory
	@echo "🧹 Cleaning backend build artifacts..."
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name ".mypy_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@rm -rf dist/ build/ .coverage htmlcov/ .tox/ .ruff_cache/
	@echo "✅ Backend cleanup completed!"

clean-logs: ## Clean log files in backend directory
	@echo "🧹 Cleaning backend log files..."
	@rm -f logs.log *.log
	@echo "✅ Backend log files cleaned!"

# Docker operations (if needed)
docker-build: ## Build Docker image
	@echo "🐳 Building Docker image..."
	docker build -t tai-backend .

docker-run: ## Run Docker container
	@echo "🐳 Running Docker container..."
	docker run -p 8000:8000 --env-file .env tai-backend

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
	@echo "Root Directory: $(ROOT_DIR)"
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
	@echo "✅ All backend quality checks passed!"

ci: install lint test ## Run CI pipeline locally
	@echo "✅ Backend CI pipeline completed successfully!"
