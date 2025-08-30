# TAI Backend Service

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.115+-green.svg)](https://fastapi.tiangolo.com/)
[![Poetry](https://img.shields.io/badge/poetry-1.4+-blue.svg)](https://python-poetry.org/)

FastAPI backend service for TAI (Teaching Assistant Intelligence) with RAG capabilities, course management, and file services.

## 🚀 Quick Start

### Development Setup (macOS/Windows)

```bash
# Install dependencies (without GPU)
make install

# Initialize database (loads from MongoDB if empty)
make db-init

# Start development server
make dev
```

### Production Setup (Linux with NVIDIA GPU)

```bash
# Install with GPU dependencies (vLLM)
make install-gpu

# Configure environment for production
cp .env.example .env
# Edit .env: set environment=production, configure MongoDB URI

# Initialize database
make db-init

# Start production server
make server
```

Visit `http://localhost:8000` for API documentation.

## 🏗️ Architecture

- **Framework**: FastAPI with SQLAlchemy ORM
- **Database**: Hybrid SQLite (local) + MongoDB (cloud backup)
- **AI Models**: BGE-M3 embeddings, vLLM (production), OpenAI/Remote APIs
- **Authentication**: JWT token-based with Google OAuth support

## 📦 Installation Options

```bash
# Development (macOS/Windows compatible)
make install          # Core dependencies, no GPU support

# Production (Linux servers with NVIDIA GPU)
make install-gpu      # Includes vLLM for local model inference

# Complete environment (all features)
make install-full     # Development + GPU dependencies
```

## ⚙️ Environment Configuration

Create `.env` from `.env.example` and configure:

```bash
# Environment
environment=dev              # dev, production, test

# Database
DATA_DIR=/path/to/course/files
MONGODB_URI=mongodb+srv://user:pass@cluster.mongodb.net/
MONGODB_ENABLED=true

# LLM Configuration
llm_mode=remote              # local (vLLM), remote (API), mock (dev)
remote_model_url=https://your-api-endpoint.com

# Server
HOST=127.0.0.1
PORT=8000
RELOAD=true                  # false for production
```

## 🛠️ Development Commands

```bash
# Setup
make install              # Install dependencies (dev)
make db-init              # Initialize database

# Development
make dev                  # Start with auto-reload
make test                 # Run test suite
make lint                 # Code linting (ruff)
make format              # Auto-format code

# Package management
make add PKG=package-name    # Add dependency
make remove PKG=package-name # Remove dependency

# Database
make admin               # Open admin interface (:8000/admin)
```

## 🗄️ Database System

### Hybrid Architecture
- **Local SQLite**: `db/courses.db`, `db/metadata.db` for fast access
- **MongoDB Cloud**: Backup and synchronization
- **Auto-initialization**: Loads from MongoDB when local DBs are empty

### Key Workflows

```bash
# Fresh setup from MongoDB
make db-init

# Export local data to MongoDB
python scripts/export_sqlite_data.py
python scripts/seed_mongodb.py

# Disaster recovery
rm -f db/*.db
python scripts/initialize_db_and_files.py --force
```

## 🌐 Core API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/courses` | GET | List courses with pagination |
| `/api/files` | GET | List files with filtering |
| `/api/files/{file_id}/download` | GET | Download file by UUID |
| `/api/completions` | POST | Generate chat completions with RAG |
| `/admin/` | GET | Database administration interface |
| `/health` | GET | Health check |

## 🚀 Production Deployment

### Docker Deployment

```bash
# Build and deploy
docker-compose up --build
```

### Manual Production Setup

```bash
# 1. Install with GPU support
make install-gpu

# 2. Configure production environment
export RELOAD=false
export LOG_LEVEL=info
export HOST=0.0.0.0
export environment=production

# 3. Start with gunicorn
poetry run gunicorn -w 4 -k uvicorn.workers.UvicornWorker main:app
```

### Production Environment Variables

```bash
# Required for production
environment=production
llm_mode=local              # Uses vLLM for local inference
MONGODB_URI=mongodb+srv://...
DATA_DIR=/path/to/course/files
RELOAD=false
LOG_LEVEL=info
HOST=0.0.0.0
```

## 🔧 Troubleshooting

### Common Issues

**Installation fails on macOS**
```bash
# Use development installation (no GPU dependencies)
make install
```

**vLLM import errors**
```bash
# Check environment - vLLM only loads in production
python -c "from app.config import settings; print(settings.effective_llm_mode)"
```

**Database empty after setup**
```bash
# Check MongoDB connection and reinitialize
python scripts/seed_mongodb.py --status
make db-init
```

**Model loading fails**
```bash
# Verify GPU availability (production only)
python -c "import torch; print(torch.cuda.is_available())"
```

## 🤝 Contributing

1. Use `make install` for development setup
2. Run `make lint` and `make test` before committing
3. Follow existing code patterns and add tests
4. Update API documentation for endpoint changes

## 📚 Documentation

- [TAI Project Overview](../../README.md)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Development Commands Reference](./Makefile)