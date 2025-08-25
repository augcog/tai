# Claude Backend Documentation

This file provides specialized guidance for working with the AI Chatbot Backend component of the TAI monorepo.

## Component Overview

The AI Chatbot Backend is a FastAPI service with RAG capabilities, course management, and file services. Located in `/ai_chatbot_backend/`, it provides the main web service for the TAI platform.

## Architecture

### Core Technologies
- **Framework**: FastAPI with SQLAlchemy ORM
- **Database**: Hybrid SQLite (local) + MongoDB (cloud) architecture
- **AI/ML**: FlagEmbedding (BGE-M3), Transformers, OpenAI integration
- **Authentication**: JWT token-based authentication
- **Admin Interface**: SQLAdmin for database management

### Database Architecture
The system uses a **hybrid database approach**:
- **Local SQLite**: `db/courses.db` and `db/metadata.db` for fast access
- **MongoDB Cloud**: Backup and synchronization with cloud storage
- **Auto-initialization**: Loads from MongoDB when local databases are empty

Database mapping configuration is in `config/database_mapping.json`.

## Key File Locations

### Configuration Files
- `ai_chatbot_backend/.env` - Backend environment configuration
- `ai_chatbot_backend/config/database_mapping.json` - Database mapping configuration
- `ai_chatbot_backend/course.json` - Fallback course configuration

### Entry Points
- `ai_chatbot_backend/main.py` - FastAPI application entry point
- `ai_chatbot_backend/app/api/router.py` - API route definitions

### Core Modules
- `ai_chatbot_backend/app/services/` - Business logic and RAG services
- `ai_chatbot_backend/app/core/models/` - Database models
- `ai_chatbot_backend/app/schemas/` - Pydantic schemas
- `ai_chatbot_backend/app/admin/` - Admin interface

### Database Scripts
- `ai_chatbot_backend/scripts/initialize_db_and_files.py` - Database initialization
- `ai_chatbot_backend/scripts/seed_mongodb.py` - MongoDB seeding
- `ai_chatbot_backend/scripts/export_sqlite_data.py` - SQLite data export

## Development Commands

**Navigate to `ai_chatbot_backend/` directory first**

### Setup and Installation
```bash
make install              # Install dependencies in local .venv
make db-init              # Initialize database (loads from MongoDB if empty)
```

### Development
```bash
make dev                  # Start development server (http://localhost:8000)
make server              # Start production server
```

### Testing
```bash
make test                # Run all tests
make test-unit           # Run unit tests only
make test-integration    # Run integration tests only
make test-api           # Run API tests
make coverage            # Run tests with coverage report
```

### Code Quality
```bash
make lint                # Run linting checks (ruff)
make format              # Format code and auto-fix issues
```

### Database Management
```bash
make db-status           # Check database status
make admin               # Open admin interface (http://localhost:8000/admin)
```

## API Endpoints

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
- `/health` - Health check endpoint
- `/database-status` - Database status and statistics

## Environment Configuration

### Required Variables
```bash
# Database
DATA_DIR=/path/to/course/files
MONGODB_URI=mongodb+srv://...

# AI Models
LLM_MODE=remote  # or local, openai, anthropic

# Server
HOST=localhost
PORT=8000
RELOAD=true

# Authentication (optional)
SECRET_KEY=your-secret-key
API_TOKENS=token1,token2
```

## Database Workflows

### Workflow 1: Seeding MongoDB from Local Data
```bash
# Export local SQLite data to JSON files
python scripts/export_sqlite_data.py

# Seed MongoDB with the exported data
python scripts/seed_mongodb.py

# Verify MongoDB contains the data
python scripts/seed_mongodb.py --status
```

### Workflow 2: Fresh Installation from MongoDB
```bash
# Configure MongoDB URI in .env
cp .env.example .env

# Initialize databases (will load from MongoDB)
make db-init

# Start the server
make dev
```

### Workflow 3: Disaster Recovery
```bash
# Remove corrupted databases
rm -f db/courses.db db/metadata.db

# Force reinitialize from MongoDB
python scripts/initialize_db_and_files.py --force
```

## Testing Strategy

### Test Structure
```
tests/
├── unit_tests/          # Unit tests for individual components
├── integration_tests/   # Integration tests for API endpoints
├── fixtures/            # Test fixtures and example data
└── common/              # Shared test utilities
```

### Test Categories
- **Unit Tests**: Individual component testing
- **Integration Tests**: API endpoint testing with real database
- **API Tests**: Full request/response cycle testing
- **Test Fixtures**: Comprehensive request/response examples

## Development Patterns

### Error Handling
- Use FastAPI's `HTTPException` for API errors
- Implement proper logging with structured formats
- Handle MongoDB connectivity gracefully with fallbacks

### Database Patterns
- Use SQLAlchemy ORM models in `app/core/models/`
- Implement service layer in `app/services/` for business logic
- Use Pydantic schemas in `app/schemas/` for request/response validation

### RAG Development
- RAG services in `app/services/rag_*.py`
- Embedding generation with BGE-M3
- Integration with RAG pipeline component

## Common Issues & Troubleshooting

### Database Issues
```bash
# Check database status
python scripts/initialize_db_and_files.py --check

# Reset local databases
rm -f db/courses.db db/metadata.db
make db-init

# Force reinitialize from MongoDB
python scripts/initialize_db_and_files.py --force
```

### MongoDB Connection Issues
```bash
# Test MongoDB connection
python scripts/seed_mongodb.py --status

# Check MongoDB URI in .env
grep MONGODB_URI .env

# Validate MongoDB data
python scripts/seed_mongodb.py --dry-run
```

### Authentication Errors
```bash
# Validate API tokens
curl -H "Authorization: Bearer your-token" http://localhost:8000/health
```

## Security Features

1. **API Token Authentication**: All endpoints require valid tokens
2. **UUID-based File Access**: No file path exposure
3. **Directory Traversal Protection**: Secure file serving
4. **CORS Configuration**: Configurable origins
5. **Input Validation**: Pydantic models for all requests

## Performance Optimizations

- **Auto-discovery Caching**: Prevents repeated file scanning
- **Database Indexing**: Optimized queries for common operations
- **Streaming Responses**: Real-time chat completions
- **Embedding Caching**: Reuse of computed embeddings
- **Connection Pooling**: Efficient database connections