# TAI Backend Service

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.115+-green.svg)](https://fastapi.tiangolo.com/)
[![Poetry](https://img.shields.io/badge/poetry-1.4+-blue.svg)](https://python-poetry.org/)

FastAPI backend service for TAI (Teaching Assistant Intelligence) with RAG capabilities, course management, and file services.

## 🚀 Quick Start

```bash
# Install dependencies
make install

# Start development server
make dev
```

Visit `http://localhost:8000` to see the API documentation.

## 🏗️ Architecture

The TAI backend is built with modern Python technologies:

- **Framework**: FastAPI with SQLAlchemy ORM
- **Database**: SQLite with optional vector extensions (sqlite-vss)
- **AI/ML**: FlagEmbedding (BGE-M3), Transformers, OpenAI integration
- **Authentication**: JWT token-based authentication
- **Admin Interface**: SQLAdmin for database management

## 📁 Project Structure

```
ai_chatbot_backend/
├── app/
│   ├── api/routes/          # API endpoints
│   ├── core/models/         # Database models
│   ├── services/            # Business logic
│   ├── schemas/             # Pydantic schemas
│   └── admin/               # Admin interface
├── tests/                   # Test suites
├── data/                    # Course files and documents
├── scripts/                 # Utility scripts
└── static/templates/        # Frontend assets
```

## 🎯 Core Features

### 1. **Course Management**

- Course registration with metadata (name, code, semester, school)
- Access control (public, login_required, private)
- Course ordering and enabling/disabling
- Pagination support

### 2. **File System Service**

- **Auto-discovery**: Automatically scans and registers new files
- **UUID-based access**: Secure file access without exposing paths
- **Metadata extraction**: Course code, category, and title from file paths
- **File categorization**: Documents, videos, audio, other
- **Search and filtering**: By course, category, filename, and title

### 3. **AI & RAG System**

- **Embedding Model**: BAAI/bge-m3 (BGE-M3) with fp16 optimization
- **Multi-modal scoring**: Dense, sparse, and ColBERT embeddings
- **Course-specific knowledge**: Separate embeddings per course
- **Streaming completions**: Real-time response generation

## 🛠️ Development Commands

### Installation & Setup

```bash
make install           # Install all dependencies
make install-dev       # Install with development tools
make clean             # Clean build artifacts
```

### Development Server

```bash
make dev               # Start development server with auto-reload
make server            # Start production server
make db-init           # Initialize database and files
```

### Testing

```bash
make test              # Run all tests
make test-unit         # Run unit tests only
make test-integration  # Run integration tests only
make test-api          # Run API tests
make coverage          # Run tests with coverage report
```

### Code Quality

```bash
make lint              # Run linting (ruff + mypy)
make format            # Format code (black + ruff)
make check             # Run all quality checks
```

## 📦 Package Management

TAI Backend uses the unified monorepo Poetry environment. All package management commands automatically modify the root `pyproject.toml`:

```bash
# Add new packages (adds to root monorepo)
make add PKG=torch                    # Add production dependency to root
make add-dev PKG=pytest              # Add development dependency to root

# Manage packages (all modify root pyproject.toml)
make remove PKG=torch                 # Remove package from root
make update                          # Update all dependencies in root
make show PKG=torch                  # Show package info from root

# Installation (always installs to root .venv)
make install                         # Install all dependencies to root
make install-dev                     # Install with development dependencies
```

**Note**: This component uses the unified monorepo environment. All dependencies are managed in the root `pyproject.toml` and installed to `/tai/.venv`. The local `pyproject.toml` serves as documentation of backend-specific dependencies.

## ⚙️ Configuration

### Environment Setup

1. Copy the example environment file:

   ```bash
   cp .env.example .env
   ```

2. Configure your environment variables:

   ```bash
   # Database
   DATA_DIR=/path/to/course/files

   # AI Models
   LLM_MODE=local  # or openai, anthropic

   # Server
   HOST=localhost
   PORT=8000
   RELOAD=true

   # Authentication (optional)
   SECRET_KEY=your-secret-key
   API_TOKENS=token1,token2
   ```

### Course Configuration

Initialize courses from `course.json`:

```json
{
  "courses": [
    {
      "course_name": "Structure and Interpretation of Computer Programs",
      "course_code": "CS61A",
      "server_url": "https://example.com",
      "semester": "Fall 2024",
      "access_type": "public",
      "enabled": true,
      "order": 1
    }
  ]
}
```

### SQLite Vector Extensions (Optional)

For enhanced vector search capabilities:

1. Download `vector0.dylib` and `vss0.dylib` from [sqlite-vss releases](https://github.com/asg017/sqlite-vss/releases)
2. Place files in your system path or configure `SQLITE_VSS_PATH`
3. Enable in configuration: `USE_VECTOR_DB=true`

## 🌐 API Endpoints

### Core APIs

| Endpoint                        | Method | Description                        |
| ------------------------------- | ------ | ---------------------------------- |
| `/api/courses`                  | GET    | List courses with pagination       |
| `/api/files`                    | GET    | List files with filtering          |
| `/api/files/{file_id}`          | GET    | Get file metadata by UUID          |
| `/api/files/{file_id}/download` | GET    | Download file by UUID              |
| `/api/completions`              | POST   | Generate chat completions with RAG |
| `/api/top_k_docs`               | POST   | Retrieve relevant document chunks  |

### Admin Interface

- `/admin/` - Database administration interface
- Course and file management through web UI
- Real-time monitoring and statistics

### Health & Status

- `/health` - Health check endpoint
- `/database-status` - Database status and statistics

## 🧪 Testing

The backend includes comprehensive test suites:

### Test Structure

```
tests/
├── unit_tests/          # Unit tests for individual components
├── integration_tests/   # Integration tests for API endpoints
├── fixtures/            # Test fixtures and example data
└── common/              # Shared test utilities
```

### Running Tests

```bash
# All tests
make test

# Specific test categories
pytest tests/unit_tests/
pytest tests/integration_tests/
pytest -m "not slow"  # Skip slow tests

# With coverage
make coverage
```

### Test Data

- **Fixtures**: Request/response examples for all endpoints
- **Mock data**: Sample courses, files, and embeddings
- **Integration tests**: Full API workflow testing

## 🔒 Security Features

1. **API Token Authentication**: All endpoints require valid tokens
2. **UUID-based File Access**: No file path exposure
3. **Directory Traversal Protection**: Secure file serving
4. **CORS Configuration**: Configurable origins
5. **Input Validation**: Pydantic models for all requests

## 📊 Performance & Monitoring

### Optimizations

- **Auto-discovery Caching**: Prevents repeated file scanning
- **Database Indexing**: Optimized queries for common operations
- **Streaming Responses**: Real-time chat completions
- **Embedding Caching**: Reuse of computed embeddings
- **Connection Pooling**: Efficient database connections

### Monitoring

- Built-in health checks
- Database performance metrics
- File system statistics
- Error tracking and logging

## 🚀 Deployment

### Docker Support

```bash
# Build and run with Docker Compose
docker-compose up --build
```

### Production Configuration

```bash
# Set environment for production
export RELOAD=false
export LOG_LEVEL=info
export HOST=0.0.0.0

# Start with gunicorn (recommended for production)
poetry run gunicorn -w 4 -k uvicorn.workers.UvicornWorker main:app
```

## 🔧 Troubleshooting

### Common Issues

**Model Loading Failures**

```bash
# Check CUDA availability and memory
python -c "import torch; print(torch.cuda.is_available())"
```

**File Discovery Issues**

```bash
# Verify data directory permissions
ls -la $DATA_DIR
make db-init  # Reinitialize file registry
```

**Authentication Errors**

```bash
# Validate API tokens
curl -H "Authorization: Bearer your-token" http://localhost:8000/health
```

**Database Issues**

```bash
# Reset database
rm -f *.db
make db-init
```

### Debug Mode

```bash
# Start with debug logging
export LOG_LEVEL=debug
make dev
```

## 🤝 Contributing

1. Follow existing code structure and patterns
2. Add tests for new functionality
3. Update API documentation for changes
4. Use established error handling patterns
5. Maintain compatibility with existing course data

### Code Style

- **Linting**: Ruff + MyPy
- **Formatting**: Black (88 character line length)
- **Type Hints**: Required for all public functions
- **Docstrings**: Google style for all modules and classes

## 📚 Additional Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [SQLAlchemy ORM Guide](https://docs.sqlalchemy.org/en/20/orm/)
- [Poetry Dependency Management](https://python-poetry.org/docs/)
- [TAI Project Documentation](../../README.md)

## 🆘 Support

For issues and questions:

1. Check the troubleshooting section above
2. Review [GitHub Issues](https://github.com/your-org/tai/issues)
3. Consult the main [TAI Documentation](../../README.md)
