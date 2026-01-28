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

### Production Setup

The backend connects to external vLLM servers for AI model inference.

```bash
# 1. Start vLLM servers (follow instructions in docs/vllm-setup.md)

# 2. Install backend dependencies
make install

# 3. Configure environment for production
cp .env.example .env
# Edit .env: set environment=production, configure MongoDB URI and vLLM server URLs

# 4. Initialize database
make db-init

# 5. Start production server
make server
```

Visit `http://localhost:8000` for API documentation.

## 🏗️ Architecture

- **Framework**: FastAPI with SQLAlchemy ORM
- **Database**: Hybrid SQLite (local) + MongoDB (cloud backup)
- **AI Models**: Qwen3-Embedding, vLLM servers (OpenAI-compatible API)
- **Authentication**: JWT token-based with Google OAuth support

### Distributed Architecture

The backend is designed to run separately from the AI model servers:

```
┌─────────────────────┐     ┌─────────────────────────────────┐
│   Backend Server    │     │      GPU Server (vLLM)          │
│   (FastAPI)         │────▶│  - Chat Model (:8001)           │
│   - API endpoints   │     │  - Embedding Model (:8002)      │
│   - RAG pipeline    │     │  - Whisper Model (:8003)        │
│   - File services   │     │  - TTS Model (:8004, optional)  │
└─────────────────────┘     └─────────────────────────────────┘
```

This allows running the backend on a lightweight server while GPU-intensive
model inference runs on dedicated hardware.

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
llm_mode=local               # local (vLLM servers), remote (legacy API), mock (dev)

# Server
HOST=127.0.0.1
PORT=8000
RELOAD=true                  # false for production
```

### vLLM Server Configuration

When `llm_mode=local`, the backend connects to external vLLM servers via OpenAI-compatible APIs.
Configure the server URLs in your `.env` file:

```bash
# vLLM Server URLs (default: localhost, change IP for remote GPU servers)
VLLM_CHAT_URL=http://localhost:8001/v1        # Main chat model
VLLM_EMBEDDING_URL=http://localhost:8002/v1   # Embedding model for RAG
VLLM_WHISPER_URL=http://localhost:8003/v1     # Speech-to-text
VLLM_TTS_URL=http://localhost:8004/v1         # Text-to-speech (optional)

# API key for vLLM servers (use 'EMPTY' if no auth required)
VLLM_API_KEY=EMPTY

# Model IDs (must match models loaded on vLLM servers)
VLLM_CHAT_MODEL=cpatonn/Qwen3-30B-A3B-Thinking-2507-AWQ-4bit
VLLM_EMBEDDING_MODEL=Qwen/Qwen3-Embedding-4B
VLLM_WHISPER_MODEL=openai/whisper-large-v3
```

**Running on separate machines:** Replace `localhost` with the GPU server's IP address.
See [4090modelservice.md](docs/vllm-setup.md) for instructions on starting vLLM servers.

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
| `/api/chat/completions` | POST | Chat completions with RAG (streaming) |
| `/api/chat/tts` | POST | Text-to-speech conversion |
| `/api/chat/voice_to_text` | POST | Speech-to-text transcription |
| `/api/courses` | GET | List courses with pagination |
| `/api/files` | GET | List files with filtering |
| `/api/files/{file_id}/download` | GET | Download file by UUID |
| `/api/files/{file_id}/extra_info` | GET | File sections and concepts |
| `/api/files/browse` | GET | Directory browser |
| `/api/problems` | GET | List problems by file |
| `/admin/` | GET | Database administration interface |
| `/health` | GET | Health check |
| `/database-status` | GET | Database initialization status |

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
llm_mode=local              # Connects to vLLM servers
MONGODB_URI=mongodb+srv://...
DATA_DIR=/path/to/course/files
RELOAD=false
LOG_LEVEL=info
HOST=0.0.0.0

# vLLM servers (change localhost to remote IP if servers are on different machines)
VLLM_CHAT_URL=http://localhost:8001/v1
VLLM_EMBEDDING_URL=http://localhost:8002/v1
VLLM_WHISPER_URL=http://localhost:8003/v1
VLLM_API_KEY=your-secure-api-key
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

- [TAI Project Overview](../README.md)
- [vLLM Server Setup Guide](docs/vllm-setup.md)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Development Commands Reference](./Makefile)