# TAI Backend Service

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.115+-green.svg)](https://fastapi.tiangolo.com/)
[![Poetry](https://img.shields.io/badge/poetry-1.4+-blue.svg)](https://python-poetry.org/)

FastAPI backend service for TAI (Teaching Assistant Intelligence) with RAG capabilities, course management, and file services.

## 🚀 Quick Start

```bash
# Install dependencies
make install

# Initialize database (loads from MongoDB if empty)
make db-init

# Start development server
make dev
```

Visit `http://localhost:8000` to see the API documentation.

## 🏗️ Architecture

The TAI backend is built with modern Python technologies:

- **Framework**: FastAPI with SQLAlchemy ORM
- **Database**: SQLite (local) + MongoDB (cloud) with optional vector extensions (sqlite-vss)
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
├── scripts/                 # Database and MongoDB utility scripts
├── config/                  # Database mapping configuration
├── db/                      # Local SQLite databases
└── static/templates/        # Frontend assets
```

## 🎯 Core Features

### 1. **Course Management**

- Course registration with metadata (name, code, semester, school)
- Access control (public, login_required, private)
- Course ordering and enabling/disabling
- Pagination support

### 2. **Database System**

- **Hybrid Architecture**: Local SQLite databases with MongoDB cloud backup
- **Auto-initialization**: Loads data from MongoDB when local databases are empty
- **Separate Databases**: 
  - `courses.db` - Course information
  - `metadata.db` - File metadata and problem data
- **MongoDB Integration**: Seamless data migration and backup to cloud

### 3. **File System Service**

- **UUID-based access**: Secure file access without exposing paths
- **Metadata management**: Course code, category, and title from file paths
- **File categorization**: Documents, videos, audio, other
- **Search and filtering**: By course, category, filename, and title

### 4. **AI & RAG System**

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
make db-init           # Initialize database (loads from MongoDB if empty)
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
make lint              # Run linting checks (ruff)
make format            # Format code and auto-fix linting issues (ruff)
make check             # Run all quality checks (lint + tests)
```

### Database Management

```bash
# Database Initialization
make db-init                              # Initialize database (loads from MongoDB if empty)
python scripts/initialize_db_and_files.py --check    # Check database status
python scripts/initialize_db_and_files.py --force    # Force clear and reinitialize from MongoDB

# MongoDB Operations
python scripts/export_sqlite_data.py                 # Export local SQLite data to JSON
python scripts/export_sqlite_data.py --summary       # Show export summary
python scripts/seed_mongodb.py                       # Seed MongoDB with exported data
python scripts/seed_mongodb.py --status              # Check MongoDB connection and data
python scripts/seed_mongodb.py --dry-run             # Preview what would be seeded
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

   # MongoDB Configuration
   MONGODB_URI=mongodb+srv://username:password@cluster.mongodb.net/?retryWrites=true&w=majority&appName=yourapp
   MONGODB_ENABLED=true

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

### Database Architecture

The system uses a hybrid database approach:

#### Local Databases
- **`db/courses.db`**: Course information (SQLite)
- **`db/metadata.db`**: File metadata and problem data (SQLite)

#### MongoDB Cloud Backup
- **`courses.courses`**: Course information backup
- **`metadata.file`**: File metadata backup  
- **`metadata.problem`**: Problem data backup

### Database Initialization Flow

1. **Check Local Databases**: System checks if local SQLite databases exist and have data
2. **Load from MongoDB**: If local databases are empty, automatically loads data from MongoDB
3. **Fallback to JSON**: If MongoDB is unavailable, loads courses from `course.json`

### Course Configuration

Initialize courses from `course.json` (fallback when MongoDB is unavailable):

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

### MongoDB Setup

1. **Configure MongoDB URI**: Update `MONGODB_URI` in `.env` with your MongoDB connection string
2. **Enable MongoDB**: Set `MONGODB_ENABLED=true` in `.env`
3. **Seed MongoDB**: Use the provided scripts to migrate data from local to MongoDB

#### Initial MongoDB Seeding

```bash
# Export local SQLite data to JSON
python scripts/export_sqlite_data.py

# Seed MongoDB with exported data
python scripts/seed_mongodb.py

# Verify MongoDB data
python scripts/seed_mongodb.py --status
```

#### Fresh Installation from MongoDB

```bash
# Remove local databases (if any)
rm -f db/courses.db db/metadata.db

# Initialize from MongoDB
python scripts/initialize_db_and_files.py

# Verify local databases were created
python scripts/initialize_db_and_files.py --check
```

## 🗄️ Database Management Workflows

### Workflow 1: Seeding MongoDB from Local Data

When you have local SQLite databases with data and want to back them up to MongoDB:

```bash
# 1. Export local SQLite data to JSON files
python scripts/export_sqlite_data.py

# 2. Review what will be exported
python scripts/export_sqlite_data.py --summary

# 3. Seed MongoDB with the exported data
python scripts/seed_mongodb.py

# 4. Verify MongoDB contains the data
python scripts/seed_mongodb.py --status
```

### Workflow 2: Fresh Installation from MongoDB

When setting up a new development environment or deployment:

```bash
# 1. Configure MongoDB URI in .env
cp .env.example .env
# Edit .env and set MONGODB_URI=your_mongodb_connection_string

# 2. Install dependencies
make install

# 3. Initialize databases (will load from MongoDB)
make db-init

# 4. Verify databases were created
python scripts/initialize_db_and_files.py --check

# 5. Start the server
make dev
```

### Workflow 3: Updating MongoDB Data

When you need to update the MongoDB data with new local changes:

```bash
# 1. Export updated local data
python scripts/export_sqlite_data.py

# 2. Update MongoDB (will overwrite existing data)
python scripts/seed_mongodb.py --clean

# 3. Verify the update
python scripts/seed_mongodb.py --status
```

### Workflow 4: Disaster Recovery

When local databases are corrupted or lost:

```bash
# 1. Remove corrupted databases
rm -f db/courses.db db/metadata.db

# 2. Force reinitialize from MongoDB
python scripts/initialize_db_and_files.py --force

# 3. Verify recovery
python scripts/initialize_db_and_files.py --check
```

### Configuration Management

The database mapping is configured in `config/database_mapping.json`:

```json
{
  "mongodb": {
    "uri": "ENV:MONGODB_URI",
    "databases": {
      "courses": {
        "collections": {
          "courses": {
            "source_db": "courses.db",
            "source_table": "courses"
          }
        }
      },
      "metadata": {
        "collections": {
          "file": {
            "source_db": "metadata.db", 
            "source_table": "file"
          },
          "problem": {
            "source_db": "metadata.db",
            "source_table": "problem"
          }
        }
      }
    }
  }
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

**Database Issues**

```bash
# Check database status
python scripts/initialize_db_and_files.py --check

# Reset local databases
rm -f db/courses.db db/metadata.db
make db-init

# Force reinitialize from MongoDB
python scripts/initialize_db_and_files.py --force
```

**MongoDB Connection Issues**

```bash
# Test MongoDB connection
python scripts/seed_mongodb.py --status

# Check MongoDB URI in .env
grep MONGODB_URI .env

# Validate MongoDB data
python scripts/seed_mongodb.py --dry-run
```

**Data Migration Issues**

```bash
# Export local data to JSON
python scripts/export_sqlite_data.py --summary

# Seed MongoDB with fresh data
python scripts/seed_mongodb.py --clean

# Verify data integrity
python scripts/seed_mongodb.py --status
```

**Authentication Errors**

```bash
# Validate API tokens
curl -H "Authorization: Bearer your-token" http://localhost:8000/health
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

- **Linting & Formatting**: Ruff (88 character line length)
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
